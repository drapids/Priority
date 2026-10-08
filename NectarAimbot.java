package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class NectarAimbot {

    public static final double CAMERA_HEIGHT_IN = 8.0;
    public static final double CAMERA_PITCH_DEG = 20.0;
    public static final double CAMERA_HFOV_DEG = 60.0;
    public static final double CAMERA_OFFSET_IN = 0.0;

    private static final float ACQUIRE_CONFIDENCE = 0.55f;
    private static final double MAX_RANGE_IN = 120.0;
    private static final double GATE_IN = 12.0;
    private static final double SMOOTHING = 0.6;
    private static final double LOST_TIME = 0.6;
    private static final double STALE_TIME = 0.5;

    private static final double KP_TURN = 1.6;
    private static final double KD_TURN = 0.08;
    private static final double MAX_TURN = 0.6;
    private static final double KP_DRIVE = 0.02;
    private static final double MIN_DRIVE = 0.18;
    private static final double MAX_DRIVE = 0.7;
    private static final double ALIGN_RAD = Math.toRadians(35);

    private static final double INTAKE_RANGE_IN = 24.0;
    private static final double COLLECT_RANGE_IN = 14.0;
    private static final double COAST_TIME = 0.45;
    private static final double COAST_POWER = 0.3;

    private static final int HISTORY = 64;

    public double forward, turn;
    public boolean active, wantIntake, coasting;
    public double range, bearingDeg;
    public float confidence;

    private final int targetLabel;
    private final long[] historyNanos = new long[HISTORY];
    private final double[] historyHeading = new double[HISTORY];
    private int historyNext, historyCount;

    private boolean locked;
    private double trackRange, trackHeading;
    private long lastFrameNanos;
    private double lastError;
    private boolean hasLastError;

    private final ElapsedTime sinceSeen = new ElapsedTime();
    private final ElapsedTime coastTimer = new ElapsedTime();
    private final ElapsedTime dtTimer = new ElapsedTime();

    public NectarAimbot(int targetLabel) {
        this.targetLabel = targetLabel;
    }

    public void reset() {
        locked = false;
        coasting = false;
        active = false;
        wantIntake = false;
        hasLastError = false;
        forward = 0;
        turn = 0;
    }

    public void update(NectarVision.Frame frame, double heading) {
        long now = System.nanoTime();
        record(now, heading);

        boolean fresh = frame != null && (now - frame.captureNanos) / 1e9 < STALE_TIME;
        if (fresh && frame.captureNanos != lastFrameNanos) {
            lastFrameNanos = frame.captureNanos;
            ingest(frame);
        }

        if (locked && sinceSeen.seconds() > LOST_TIME) {
            locked = false;
            hasLastError = false;
            if (trackRange < COLLECT_RANGE_IN) {
                coasting = true;
                coastTimer.reset();
            }
        }

        double dt = Math.max(1e-3, dtTimer.seconds());
        dtTimer.reset();

        if (locked) {
            coasting = false;
            double error = AngleUnit.normalizeRadians(trackHeading - heading);
            double dError = hasLastError ? (error - lastError) / dt : 0;
            lastError = error;
            hasLastError = true;

            double align = Range.clip(1.0 - Math.abs(error) / ALIGN_RAD, 0, 1);
            turn = Range.clip(-(KP_TURN * error + KD_TURN * dError), -MAX_TURN, MAX_TURN);
            forward = Range.clip(KP_DRIVE * trackRange + MIN_DRIVE, 0, MAX_DRIVE) * align;
            wantIntake = trackRange < INTAKE_RANGE_IN;
            active = true;
            range = trackRange;
            bearingDeg = Math.toDegrees(error);
        } else if (coasting && coastTimer.seconds() < COAST_TIME) {
            turn = 0;
            forward = COAST_POWER;
            wantIntake = true;
            active = true;
        } else {
            coasting = false;
            turn = 0;
            forward = 0;
            wantIntake = false;
            active = false;
        }
    }

    private void ingest(NectarVision.Frame frame) {
        double captureHeading = headingAt(frame.captureNanos);
        double fx = (frame.width / 2.0) / Math.tan(Math.toRadians(CAMERA_HFOV_DEG) / 2.0);

        double trackX = trackRange * Math.cos(trackHeading);
        double trackY = trackRange * Math.sin(trackHeading);

        double bestScore = Double.MAX_VALUE;
        double bestRange = 0, bestHeading = 0;
        float bestConfidence = 0;

        for (NectarVision.Detection d : frame.detections) {
            if (d.label != targetLabel) continue;
            if (!locked && d.confidence < ACQUIRE_CONFIDENCE) continue;

            double u = d.centerX() - frame.width / 2.0;
            double v = d.bottom - frame.height / 2.0;
            double depression = Math.toRadians(CAMERA_PITCH_DEG) + Math.atan2(v, fx);
            if (depression <= Math.toRadians(1)) continue;

            double ground = CAMERA_HEIGHT_IN / Math.tan(depression);
            double bearing = Math.atan2(u, fx);
            double forwardIn = ground + CAMERA_OFFSET_IN;
            double lateralIn = ground * Math.tan(bearing);
            double r = Math.hypot(forwardIn, lateralIn);
            if (r > MAX_RANGE_IN) continue;

            double absHeading = captureHeading - Math.atan2(lateralIn, forwardIn);

            double score;
            if (locked) {
                score = Math.hypot(r * Math.cos(absHeading) - trackX, r * Math.sin(absHeading) - trackY);
                if (score > GATE_IN) continue;
            } else {
                score = r;
            }

            if (score < bestScore) {
                bestScore = score;
                bestRange = r;
                bestHeading = absHeading;
                bestConfidence = d.confidence;
            }
        }

        if (bestScore == Double.MAX_VALUE) return;

        if (locked) {
            trackRange += SMOOTHING * (bestRange - trackRange);
            trackHeading += SMOOTHING * AngleUnit.normalizeRadians(bestHeading - trackHeading);
            trackHeading = AngleUnit.normalizeRadians(trackHeading);
        } else {
            trackRange = bestRange;
            trackHeading = bestHeading;
            locked = true;
            hasLastError = false;
        }
        confidence = bestConfidence;
        sinceSeen.reset();
    }

    private void record(long nanos, double heading) {
        historyNanos[historyNext] = nanos;
        historyHeading[historyNext] = heading;
        historyNext = (historyNext + 1) % HISTORY;
        historyCount = Math.min(historyCount + 1, HISTORY);
    }

    private double headingAt(long nanos) {
        double best = 0;
        long bestGap = Long.MAX_VALUE;
        for (int i = 0; i < historyCount; i++) {
            long gap = Math.abs(historyNanos[i] - nanos);
            if (gap < bestGap) {
                bestGap = gap;
                best = historyHeading[i];
            }
        }
        return best;
    }

    public boolean isLocked() {
        return locked;
    }
}
