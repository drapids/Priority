# Priority
priority is the pinnacle of code

## OpModes
- **Priority**: TeleOp
- **Priority Auto**: shoot preloads into the HIVE, then LEAVE

## Nectar aimbot
Gamepad1 dpad right toggles it. It finds the nearest NECTAR of our alliance with a YOLO model on the webcam, turns to it, drives in and runs the intake. Moving any stick takes control back instantly.

### Setup
1. Robot config: webcam named `Webcam 1`.
2. `TeamCode/build.gradle` dependencies:
   ```
   implementation 'org.tensorflow:tensorflow-lite:2.16.1'
   ```
3. Train the model. Classes, in this order: `pollen`, `red_nectar`, `blue_nectar`.
   - Take 300+ pictures from the robot webcam: different lighting, distances, field spots, with other robots, wheels and people in frame so it learns what is *not* nectar.
   - Label them (Roboflow or CVAT), export in YOLO format.
   - Train and export:
     ```
     pip install ultralytics
     yolo detect train model=yolov8n.pt data=data.yaml imgsz=320 epochs=100
     yolo export model=runs/detect/train/weights/best.pt format=tflite imgsz=320
     ```
4. Rename `best_float32.tflite` to `nectar.tflite` and copy it to `/sdcard/FIRST/tflitemodels/` on the Control Hub.
5. Measure the camera and set `CAMERA_HEIGHT_IN`, `CAMERA_PITCH_DEG`, `CAMERA_HFOV_DEG` in `NectarAimbot.java`.

Init telemetry shows `nectar vision: ready` when everything is found.
