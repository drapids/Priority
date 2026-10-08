package org.firstinspires.ftc.teamcode;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.tensorflow.lite.DataType;
import org.tensorflow.lite.Interpreter;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;
import java.nio.MappedByteBuffer;
import java.nio.channels.FileChannel;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class NectarVision implements VisionProcessor {

    public static final String[] LABELS = {"pollen", "red_nectar", "blue_nectar"};
    public static final int POLLEN = 0;
    public static final int RED_NECTAR = 1;
    public static final int BLUE_NECTAR = 2;

    private static final float NMS_IOU = 0.5f;

    public static class Detection {
        public final int label;
        public final float confidence;
        public final float left, top, right, bottom;

        Detection(int label, float confidence, float left, float top, float right, float bottom) {
            this.label = label;
            this.confidence = confidence;
            this.left = left;
            this.top = top;
            this.right = right;
            this.bottom = bottom;
        }

        public float centerX() {
            return (left + right) / 2f;
        }

        public float area() {
            return Math.max(0, right - left) * Math.max(0, bottom - top);
        }

        float iou(Detection o) {
            float w = Math.min(right, o.right) - Math.max(left, o.left);
            float h = Math.min(bottom, o.bottom) - Math.max(top, o.top);
            if (w <= 0 || h <= 0) return 0;
            float inter = w * h;
            return inter / (area() + o.area() - inter);
        }
    }

    public static class Frame {
        public final List<Detection> detections;
        public final long captureNanos;
        public final int width, height;

        Frame(List<Detection> detections, long captureNanos, int width, int height) {
            this.detections = detections;
            this.captureNanos = captureNanos;
            this.width = width;
            this.height = height;
        }
    }

    private final Interpreter interpreter;
    private final int inputSize;
    private final int rows, cols;
    private final boolean attributesFirst;
    private final float minConfidence;
    private final ByteBuffer input;
    private final FloatBuffer inputFloats;
    private final float[] pixels;
    private final float[][][] output;

    private final Mat rgb = new Mat();
    private final Mat resized = new Mat();
    private final Mat padded = new Mat();
    private final Mat floats = new Mat();
    private final Scalar padColor = new Scalar(114, 114, 114);

    private final Paint boxPaint = new Paint();
    private final Paint textPaint = new Paint();

    private volatile Frame latest;
    private volatile double inferenceMs;

    public NectarVision(File model, float minConfidence, int threads) throws IOException {
        MappedByteBuffer buffer;
        try (FileInputStream stream = new FileInputStream(model); FileChannel channel = stream.getChannel()) {
            buffer = channel.map(FileChannel.MapMode.READ_ONLY, 0, channel.size());
        }

        Interpreter.Options options = new Interpreter.Options();
        options.setNumThreads(threads);
        interpreter = new Interpreter(buffer, options);

        if (interpreter.getInputTensor(0).dataType() != DataType.FLOAT32) {
            throw new IllegalStateException("model input must be float32");
        }

        int[] in = interpreter.getInputTensor(0).shape();
        inputSize = in[1];

        int[] out = interpreter.getOutputTensor(0).shape();
        rows = out[1];
        cols = out[2];
        attributesFirst = rows < cols;
        int attributes = attributesFirst ? rows : cols;
        if (attributes != 4 + LABELS.length) {
            throw new IllegalStateException("model has " + (attributes - 4) + " classes, expected " + LABELS.length);
        }

        this.minConfidence = minConfidence;
        input = ByteBuffer.allocateDirect(4 * inputSize * inputSize * 3).order(ByteOrder.nativeOrder());
        inputFloats = input.asFloatBuffer();
        pixels = new float[inputSize * inputSize * 3];
        output = new float[1][rows][cols];

        boxPaint.setStyle(Paint.Style.STROKE);
        textPaint.setColor(Color.WHITE);
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        long start = System.nanoTime();

        if (frame.channels() == 4) {
            Imgproc.cvtColor(frame, rgb, Imgproc.COLOR_RGBA2RGB);
        } else {
            frame.copyTo(rgb);
        }

        int frameW = rgb.cols();
        int frameH = rgb.rows();
        float scale = Math.min((float) inputSize / frameW, (float) inputSize / frameH);
        int newW = Math.round(frameW * scale);
        int newH = Math.round(frameH * scale);
        int padX = (inputSize - newW) / 2;
        int padY = (inputSize - newH) / 2;

        Imgproc.resize(rgb, resized, new Size(newW, newH), 0, 0, Imgproc.INTER_LINEAR);
        Core.copyMakeBorder(resized, padded, padY, inputSize - newH - padY, padX, inputSize - newW - padX,
                Core.BORDER_CONSTANT, padColor);
        padded.convertTo(floats, CvType.CV_32FC3, 1.0 / 255.0);
        floats.get(0, 0, pixels);

        inputFloats.rewind();
        inputFloats.put(pixels);
        input.rewind();
        interpreter.run(input, output);

        List<Detection> candidates = new ArrayList<>();
        int boxes = attributesFirst ? cols : rows;
        for (int i = 0; i < boxes; i++) {
            int best = -1;
            float bestScore = minConfidence;
            for (int c = 0; c < LABELS.length; c++) {
                float s = value(4 + c, i);
                if (s > bestScore) {
                    bestScore = s;
                    best = c;
                }
            }
            if (best < 0) continue;

            float cx = value(0, i);
            float cy = value(1, i);
            float w = value(2, i);
            float h = value(3, i);
            if (Math.max(Math.max(cx, cy), Math.max(w, h)) <= 1.5f) {
                cx *= inputSize;
                cy *= inputSize;
                w *= inputSize;
                h *= inputSize;
            }

            float left = clamp((cx - w / 2f - padX) / scale, frameW);
            float top = clamp((cy - h / 2f - padY) / scale, frameH);
            float right = clamp((cx + w / 2f - padX) / scale, frameW);
            float bottom = clamp((cy + h / 2f - padY) / scale, frameH);
            if (right - left < 2 || bottom - top < 2) continue;

            candidates.add(new Detection(best, bestScore, left, top, right, bottom));
        }

        Collections.sort(candidates, (a, b) -> Float.compare(b.confidence, a.confidence));
        List<Detection> kept = new ArrayList<>();
        for (Detection d : candidates) {
            boolean overlaps = false;
            for (Detection k : kept) {
                if (d.iou(k) > NMS_IOU) {
                    overlaps = true;
                    break;
                }
            }
            if (!overlaps) kept.add(d);
        }

        Frame result = new Frame(Collections.unmodifiableList(kept), captureTimeNanos, frameW, frameH);
        latest = result;
        inferenceMs = (System.nanoTime() - start) / 1e6;
        return result;
    }

    private float value(int attribute, int box) {
        return attributesFirst ? output[0][attribute][box] : output[0][box][attribute];
    }

    private static float clamp(float v, float max) {
        return Math.max(0, Math.min(max, v));
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        if (!(userContext instanceof Frame)) return;
        Frame frame = (Frame) userContext;

        boxPaint.setStrokeWidth(4 * scaleCanvasDensity);
        textPaint.setTextSize(28 * scaleCanvasDensity);

        for (Detection d : frame.detections) {
            boxPaint.setColor(d.label == RED_NECTAR ? Color.RED : d.label == BLUE_NECTAR ? Color.BLUE : Color.YELLOW);
            float l = d.left * scaleBmpPxToCanvasPx;
            float t = d.top * scaleBmpPxToCanvasPx;
            canvas.drawRect(l, t, d.right * scaleBmpPxToCanvasPx, d.bottom * scaleBmpPxToCanvasPx, boxPaint);
            canvas.drawText(String.format("%s %.2f", LABELS[d.label], d.confidence), l, t - 6 * scaleCanvasDensity, textPaint);
        }
    }

    public Frame latest() {
        return latest;
    }

    public double inferenceMs() {
        return inferenceMs;
    }

    public void close() {
        interpreter.close();
    }
}
