package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import android.util.Size;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.vision.VisionPortal;

import java.io.File;

import java.util.List;

@TeleOp(name = "Priority", group = "Competition")
public class Priority extends OpMode {

    private static final double TELEOP_LENGTH = 120;
    private static final double ENDGAME_START = 60;
    private static final double PARK_WARNING = 15;

    private static final double DEADZONE = 0.05;
    private static final double SLOW_SCALE = 0.35;
    private static final double STRAFE_FIX = 1.1;
    private static final double SLEW_RATE = 6.0;

    private static final double TICKS_PER_REV = 28.0;
    private static final double RPM_HIVE_NEAR = 2400;
    private static final double RPM_HIVE_FAR = 3100;
    private static final double RPM_FLOWER = 1200;
    private static final double RPM_TRIM_STEP = 50;
    private static final double RPM_MIN = 800;
    private static final double RPM_MAX = 4500;
    private static final double RPM_TOLERANCE = 80;
    private static final double READY_SETTLE = 0.08;
    private static final double SHOT_DIP_RPM = 150;

    private static final double LAUNCHER_P = 12;
    private static final double LAUNCHER_I = 3;
    private static final double LAUNCHER_D = 0;
    private static final double LAUNCHER_F = 14;
    private static final double NOMINAL_VOLTAGE = 12.5;

    private static final double FEED_POWER = 1.0;
    private static final double FEED_MAX_TIME = 0.35;
    private static final double FEED_COOLDOWN = 0.15;

    private static final double JAM_AMPS = 4.5;
    private static final double JAM_TRIGGER_TIME = 0.25;
    private static final double JAM_REVERSE_TIME = 0.3;
    private static final double JAM_REVERSE_POWER = -0.6;

    private static final int MAX_HELD = 4;
    private static final double BALL_PRESENT_CM = 4.0;
    private static final float COLOR_GAIN = 8f;

    private static final String MODEL_FILE = "tflitemodels/nectar.tflite";
    private static final float MODEL_CONFIDENCE = 0.4f;
    private static final int MODEL_THREADS = 4;
    private static final double AIM_OVERRIDE = 0.3;

    private static final double VOLTAGE_PERIOD = 0.5;
    private static final double CURRENT_PERIOD = 0.1;

    private enum Shooter { IDLE, SPINUP, READY, FEED, RECOVER }
    private enum Target { HIVE_NEAR, HIVE_FAR, FLOWER }
    private enum Ball { NONE, POLLEN, RED_NECTAR, BLUE_NECTAR, UNKNOWN }
    private enum Alliance { RED, BLUE }

    private DcMotor fl, fr, bl, br;
    private DcMotorEx intake, launcher;
    private CRServo leftIntakeServo, rightIntakeServo, feedServo;
    private VoltageSensor battery;
    private IMU imu;
    private NormalizedColorSensor ballColor;
    private DistanceSensor ballDistance;
    private List<LynxModule> hubs;

    private VisionPortal portal;
    private NectarVision vision;
    private NectarAimbot aimbot;
    private String visionStatus = "off";
    private boolean aimbotOn;
    private boolean assisting;

    private Alliance alliance = Alliance.BLUE;
    private Shooter shooter = Shooter.IDLE;
    private Target target = Target.HIVE_NEAR;
    private double targetRPM = RPM_HIVE_NEAR;
    private double rpm;
    private double feedStartRPM;
    private int shots;
    private boolean announcedReady;
    private String blockedReason = "";

    private Ball nextBall = Ball.NONE;
    private boolean ballWasPresent;
    private int held;

    private boolean slowMode;
    private boolean fieldCentric;
    private double heading;
    private double cmdForward, cmdStrafe;
    private double flPower, frPower, blPower, brPower;

    private double voltage = NOMINAL_VOLTAGE;
    private double intakeAmps;
    private boolean unjamming;

    private boolean endgameAnnounced, parkAnnounced, fullAnnounced;

    private boolean lastSpin, lastFire, lastFieldCentric, lastResetHeading, lastResetHeld;
    private boolean lastNear, lastFar, lastFlower, lastTrimUp, lastTrimDown;
    private boolean lastAllianceRed, lastAllianceBlue;
    private boolean lastAimbot;

    private final ElapsedTime matchTimer = new ElapsedTime();
    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime shooterTimer = new ElapsedTime();
    private final ElapsedTime settleTimer = new ElapsedTime();
    private final ElapsedTime jamTimer = new ElapsedTime();
    private final ElapsedTime unjamTimer = new ElapsedTime();
    private final ElapsedTime voltageTimer = new ElapsedTime();
    private final ElapsedTime currentTimer = new ElapsedTime();
    private double loopMs;

    @Override
    public void init() {
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        fl = hardwareMap.get(DcMotor.class, "front_left_drive");
        fr = hardwareMap.get(DcMotor.class, "front_right_drive");
        bl = hardwareMap.get(DcMotor.class, "back_left_drive");
        br = hardwareMap.get(DcMotor.class, "back_right_drive");

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");

        leftIntakeServo = hardwareMap.get(CRServo.class, "left_intake_servo");
        rightIntakeServo = hardwareMap.get(CRServo.class, "right_intake_servo");
        feedServo = hardwareMap.get(CRServo.class, "feed_servo");

        battery = hardwareMap.voltageSensor.iterator().next();

        fl.setDirection(DcMotor.Direction.FORWARD);
        bl.setDirection(DcMotor.Direction.FORWARD);
        fr.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.REVERSE);
        leftIntakeServo.setDirection(DcMotorSimple.Direction.FORWARD);
        rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        feedServo.setDirection(DcMotorSimple.Direction.FORWARD);

        for (DcMotor m : new DcMotor[]{fl, fr, bl, br}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.tryGet(IMU.class, "imu");
        if (imu != null) {
            imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
            imu.resetYaw();
        }

        ballColor = hardwareMap.tryGet(NormalizedColorSensor.class, "ball_color");
        if (ballColor != null) {
            ballColor.setGain(COLOR_GAIN);
            if (ballColor instanceof DistanceSensor) ballDistance = (DistanceSensor) ballColor;
        }

        startVision();

        voltage = battery.getVoltage();
        applyLauncherGains();

        feedServo.setPower(0);
        leftIntakeServo.setPower(0);
        rightIntakeServo.setPower(0);

        telemetry.setMsTransmissionInterval(50);
        telemetry.addData("Priority", "initialized");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        boolean red = gamepad1.b || gamepad2.b;
        boolean blue = gamepad1.x || gamepad2.x;
        if (red && !lastAllianceRed) alliance = Alliance.RED;
        if (blue && !lastAllianceBlue) alliance = Alliance.BLUE;
        lastAllianceRed = red;
        lastAllianceBlue = blue;

        telemetry.addData("Priority", "waiting for start");
        telemetry.addData("alliance", "%s   (B = red, X = blue)", alliance);
        telemetry.addData("battery", "%.2f V", battery.getVoltage());
        telemetry.addData("imu", imu != null ? "found" : "missing, field centric off");
        telemetry.addData("ball sensor", ballColor != null ? "found" : "missing, nectar guard off");
        telemetry.addData("target", "%s @ %.0f rpm", target, targetRPM);
        telemetry.addData("nectar vision", visionStatus);
        telemetry.update();
    }

    @Override
    public void start() {
        shooter = Shooter.IDLE;
        shots = 0;
        held = 0;
        matchTimer.reset();
        loopTimer.reset();
        voltageTimer.reset();
        currentTimer.reset();
        if (imu != null) imu.resetYaw();
        aimbot = new NectarAimbot(alliance == Alliance.RED ? NectarVision.RED_NECTAR : NectarVision.BLUE_NECTAR);
    }

    @Override
    public void loop() {
        for (LynxModule hub : hubs) hub.clearBulkCache();

        double dt = loopTimer.seconds();
        loopTimer.reset();
        loopMs = dt * 1000.0;

        readSensors();
        matchCues();
        runAimbot();
        drive(dt);
        runIntake();
        runShooter();
        safetyStop();
        showTelemetry();
    }

    @Override
    public void stop() {
        haltEverything();
        if (portal != null) portal.close();
        if (vision != null) vision.close();
    }

    private void startVision() {
        WebcamName camera = hardwareMap.tryGet(WebcamName.class, "Webcam 1");
        File model = new File(AppUtil.ROOT_FOLDER, MODEL_FILE);
        if (camera == null) {
            visionStatus = "no Webcam 1, aimbot disabled";
            return;
        }
        if (!model.exists()) {
            visionStatus = "missing " + model.getPath();
            return;
        }
        try {
            vision = new NectarVision(model, MODEL_CONFIDENCE, MODEL_THREADS);
            portal = new VisionPortal.Builder()
                    .setCamera(camera)
                    .setCameraResolution(new Size(640, 480))
                    .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                    .addProcessor(vision)
                    .enableLiveView(true)
                    .build();
            visionStatus = "ready";
        } catch (Throwable t) {
            vision = null;
            portal = null;
            visionStatus = "failed: " + t.getMessage();
        }
    }

    private void runAimbot() {
        boolean toggle = gamepad1.dpad_right;
        if (toggle && !lastAimbot) {
            if (vision == null) {
                gamepad1.rumbleBlips(3);
            } else {
                aimbotOn = !aimbotOn;
                aimbot.reset();
                gamepad1.rumbleBlips(aimbotOn ? 1 : 2);
            }
        }
        lastAimbot = toggle;

        if (!aimbotOn || vision == null) return;

        if (ballColor != null && held >= MAX_HELD) {
            aimbot.reset();
            return;
        }

        aimbot.update(vision.latest(), heading);
    }

    private double timeLeft() {
        return Math.max(0, TELEOP_LENGTH - matchTimer.seconds());
    }

    private boolean isEndgame() {
        return timeLeft() <= ENDGAME_START;
    }

    private void readSensors() {
        rpm = launcher.getVelocity() * 60.0 / TICKS_PER_REV;

        if (voltageTimer.seconds() > VOLTAGE_PERIOD) {
            voltageTimer.reset();
            voltage = battery.getVoltage();
            if (shooter != Shooter.IDLE) applyLauncherGains();
        }

        if (currentTimer.seconds() > CURRENT_PERIOD) {
            currentTimer.reset();
            intakeAmps = intake.getCurrent(CurrentUnit.AMPS);
        }

        if (imu != null) heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        if (ballColor != null) {
            nextBall = classifyBall();
            boolean present = nextBall != Ball.NONE;
            if (present && !ballWasPresent && held < MAX_HELD) held++;
            ballWasPresent = present;
        }
    }

    private Ball classifyBall() {
        if (ballDistance != null && ballDistance.getDistance(DistanceUnit.CM) > BALL_PRESENT_CM) {
            return Ball.NONE;
        }

        NormalizedRGBA c = ballColor.getNormalizedColors();
        float[] hsv = new float[3];
        Color.colorToHSV(c.toColor(), hsv);
        float hue = hsv[0];
        float sat = hsv[1];

        if (ballDistance == null && c.alpha < 0.2f) return Ball.NONE;
        if (sat < 0.25f) return Ball.UNKNOWN;
        if (hue >= 35 && hue <= 80) return Ball.POLLEN;
        if (hue < 25 || hue > 330) return Ball.RED_NECTAR;
        if (hue >= 180 && hue <= 260) return Ball.BLUE_NECTAR;
        return Ball.UNKNOWN;
    }

    private boolean isNectar(Ball b) {
        return b == Ball.RED_NECTAR || b == Ball.BLUE_NECTAR;
    }

    private boolean isOurNectar(Ball b) {
        return (alliance == Alliance.RED && b == Ball.RED_NECTAR)
                || (alliance == Alliance.BLUE && b == Ball.BLUE_NECTAR);
    }

    private void matchCues() {
        if (!endgameAnnounced && isEndgame()) {
            endgameAnnounced = true;
            gamepad1.rumbleBlips(3);
            gamepad2.rumbleBlips(3);
        }
        if (!parkAnnounced && timeLeft() <= PARK_WARNING) {
            parkAnnounced = true;
            gamepad1.rumble(1000);
            gamepad2.rumble(1000);
        }
        if (held >= MAX_HELD) {
            if (!fullAnnounced) {
                fullAnnounced = true;
                gamepad1.rumbleBlips(2);
            }
        } else {
            fullAnnounced = false;
        }

        boolean resetHeld = gamepad2.x;
        if (resetHeld && !lastResetHeld) held = 0;
        lastResetHeld = resetHeld;
    }

    private void drive(double dt) {
        boolean fcPressed = gamepad1.y;
        if (fcPressed && !lastFieldCentric && imu != null) {
            fieldCentric = !fieldCentric;
            gamepad1.rumble(150);
        }
        lastFieldCentric = fcPressed;

        boolean resetPressed = gamepad1.x;
        if (resetPressed && !lastResetHeading && imu != null) {
            imu.resetYaw();
            heading = 0;
            gamepad1.rumbleBlips(1);
            if (aimbot != null) aimbot.reset();
        }
        lastResetHeading = resetPressed;

        slowMode = gamepad1.left_bumper;

        double forward = shape(-gamepad1.left_stick_y);
        double strafe = shape(gamepad1.left_stick_x);
        double turn = shape(gamepad1.right_stick_x);

        boolean driverInput = Math.abs(forward) + Math.abs(strafe) + Math.abs(turn) > AIM_OVERRIDE;
        assisting = aimbotOn && aimbot != null && aimbot.active && !driverInput;

        if (assisting) {
            forward = aimbot.forward;
            strafe = 0;
            turn = aimbot.turn;
        } else if (fieldCentric) {
            double cos = Math.cos(-heading);
            double sin = Math.sin(-heading);
            double rotStrafe = strafe * cos - forward * sin;
            double rotForward = strafe * sin + forward * cos;
            strafe = rotStrafe;
            forward = rotForward;
        }

        strafe *= STRAFE_FIX;

        if (slowMode && !assisting) {
            forward *= SLOW_SCALE;
            strafe *= SLOW_SCALE;
            turn *= SLOW_SCALE;
        }

        double maxStep = SLEW_RATE * dt;
        cmdForward += Range.clip(forward - cmdForward, -maxStep, maxStep);
        cmdStrafe += Range.clip(strafe - cmdStrafe, -maxStep, maxStep);

        flPower = cmdForward + cmdStrafe + turn;
        frPower = cmdForward - cmdStrafe - turn;
        blPower = cmdForward - cmdStrafe + turn;
        brPower = cmdForward + cmdStrafe - turn;

        double max = Math.max(1.0, Math.max(
                Math.max(Math.abs(flPower), Math.abs(frPower)),
                Math.max(Math.abs(blPower), Math.abs(brPower))));

        flPower /= max;
        frPower /= max;
        blPower /= max;
        brPower /= max;

        fl.setPower(flPower);
        fr.setPower(frPower);
        bl.setPower(blPower);
        br.setPower(brPower);
    }

    private double shape(double x) {
        if (Math.abs(x) < DEADZONE) return 0;
        double scaled = (Math.abs(x) - DEADZONE) / (1.0 - DEADZONE);
        return Math.copySign(scaled * scaled, x);
    }

    private void runIntake() {
        double in = Math.max(gamepad1.right_trigger, gamepad2.right_trigger);
        double out = Math.max(gamepad1.left_trigger, gamepad2.left_trigger);
        double power = in - out;
        if (power == 0 && assisting && aimbot.wantIntake) power = 1.0;

        if (unjamming) {
            if (unjamTimer.seconds() > JAM_REVERSE_TIME) {
                unjamming = false;
                jamTimer.reset();
            } else {
                setIntake(JAM_REVERSE_POWER);
                return;
            }
        }

        if (power > 0.3 && intakeAmps > JAM_AMPS) {
            if (jamTimer.seconds() > JAM_TRIGGER_TIME) {
                unjamming = true;
                unjamTimer.reset();
                gamepad1.rumbleBlips(2);
                gamepad2.rumbleBlips(2);
                setIntake(JAM_REVERSE_POWER);
                return;
            }
        } else {
            jamTimer.reset();
        }

        setIntake(power);
    }

    private void setIntake(double power) {
        intake.setPower(power);
        leftIntakeServo.setPower(power);
        rightIntakeServo.setPower(power);
    }

    private void runShooter() {
        boolean spin = gamepad1.a || gamepad2.a;
        boolean fire = gamepad1.b || gamepad2.b;
        boolean near = gamepad1.dpad_down || gamepad2.dpad_down;
        boolean far = gamepad1.dpad_up || gamepad2.dpad_up;
        boolean flower = gamepad1.dpad_left || gamepad2.dpad_left;
        boolean trimUp = gamepad2.right_bumper;
        boolean trimDown = gamepad2.left_bumper;

        if (near && !lastNear) selectTarget(Target.HIVE_NEAR, RPM_HIVE_NEAR);
        if (far && !lastFar) selectTarget(Target.HIVE_FAR, RPM_HIVE_FAR);
        if (flower && !lastFlower) selectTarget(Target.FLOWER, RPM_FLOWER);
        if (trimUp && !lastTrimUp) setRPM(targetRPM + RPM_TRIM_STEP);
        if (trimDown && !lastTrimDown) setRPM(targetRPM - RPM_TRIM_STEP);

        lastNear = near;
        lastFar = far;
        lastFlower = flower;
        lastTrimUp = trimUp;
        lastTrimDown = trimDown;

        if (spin && !lastSpin) {
            if (shooter == Shooter.IDLE) {
                spinUp();
            } else {
                spinDown();
            }
        }
        lastSpin = spin;

        boolean inBand = Math.abs(rpm - targetRPM) < RPM_TOLERANCE;
        if (!inBand) settleTimer.reset();
        boolean atSpeed = inBand && settleTimer.seconds() > READY_SETTLE;

        blockedReason = feedBlockReason();
        if (fire && !lastFire && !blockedReason.isEmpty()) {
            gamepad1.rumbleBlips(3);
            gamepad2.rumbleBlips(3);
        }
        lastFire = fire;

        switch (shooter) {
            case IDLE:
                feedServo.setPower(0);
                break;

            case SPINUP:
                feedServo.setPower(0);
                if (atSpeed) {
                    shooter = Shooter.READY;
                    if (!announcedReady) {
                        announcedReady = true;
                        gamepad1.rumble(200);
                        gamepad2.rumble(200);
                    }
                }
                break;

            case READY:
                feedServo.setPower(0);
                if (!inBand) {
                    shooter = Shooter.SPINUP;
                } else if (fire && blockedReason.isEmpty()) {
                    feedStartRPM = rpm;
                    shooterTimer.reset();
                    shooter = Shooter.FEED;
                }
                break;

            case FEED:
                feedServo.setPower(FEED_POWER);
                boolean dipped = feedStartRPM - rpm > SHOT_DIP_RPM;
                if (dipped || shooterTimer.seconds() > FEED_MAX_TIME) {
                    if (dipped) {
                        shots++;
                        if (held > 0) held--;
                    }
                    feedServo.setPower(0);
                    shooterTimer.reset();
                    shooter = Shooter.RECOVER;
                }
                break;

            case RECOVER:
                feedServo.setPower(0);
                if (shooterTimer.seconds() > FEED_COOLDOWN) {
                    shooter = atSpeed ? Shooter.READY : Shooter.SPINUP;
                }
                break;
        }
    }

    private String feedBlockReason() {
        if (target != Target.FLOWER || ballColor == null || gamepad2.y) return "";
        if (!isNectar(nextBall)) return "";
        if (!isOurNectar(nextBall)) return "their nectar would give them the flower";
        if (!isEndgame()) return "nectar in flower before endgame is a major foul";
        return "";
    }

    private void selectTarget(Target t, double rpmTarget) {
        target = t;
        setRPM(rpmTarget);
    }

    private void setRPM(double rpmTarget) {
        targetRPM = Range.clip(rpmTarget, RPM_MIN, RPM_MAX);
        if (shooter != Shooter.IDLE) {
            launcher.setVelocity(rpmToTicks(targetRPM));
            if (shooter == Shooter.READY) shooter = Shooter.SPINUP;
        }
    }

    private void spinUp() {
        applyLauncherGains();
        launcher.setVelocity(rpmToTicks(targetRPM));
        settleTimer.reset();
        announcedReady = false;
        shooter = Shooter.SPINUP;
    }

    private void spinDown() {
        launcher.setVelocity(0);
        launcher.setPower(0);
        feedServo.setPower(0);
        shooter = Shooter.IDLE;
    }

    private void applyLauncherGains() {
        double comp = NOMINAL_VOLTAGE / Math.max(voltage, 9.0);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(LAUNCHER_P, LAUNCHER_I, LAUNCHER_D, LAUNCHER_F * comp));
    }

    private double rpmToTicks(double rpmValue) {
        return rpmValue * TICKS_PER_REV / 60.0;
    }

    private void safetyStop() {
        if ((gamepad1.back && gamepad1.start) || (gamepad2.back && gamepad2.start)) {
            haltEverything();
            requestOpModeStop();
        }
    }

    private void haltEverything() {
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
        setIntake(0);
        launcher.setPower(0);
        feedServo.setPower(0);
        shooter = Shooter.IDLE;
    }

    private void showTelemetry() {
        double left = timeLeft();
        String phase = left <= PARK_WARNING ? "PARK NOW" : isEndgame() ? "ENDGAME  flowers open" : "TELEOP";
        telemetry.addData("time", "%d:%02d  %s", (int) left / 60, (int) left % 60, phase);
        telemetry.addData("alliance", alliance);
        telemetry.addLine();
        telemetry.addData("shooter", "%s  ->  %s", shooter, target);
        telemetry.addData("rpm", "target %.0f  actual %.0f", targetRPM, rpm);
        telemetry.addData("shots", shots);
        if (!blockedReason.isEmpty()) telemetry.addData("BLOCKED", blockedReason);
        telemetry.addLine();
        if (ballColor != null) {
            telemetry.addData("next ball", nextBall);
            telemetry.addData("held", "%d / %d%s", held, MAX_HELD, held >= MAX_HELD ? "  FULL" : "");
        }
        telemetry.addData("intake", "%s  %.2f A", unjamming ? "UNJAM" : "OK", intakeAmps);
        telemetry.addLine();
        if (aimbotOn) {
            String state = aimbot.isLocked()
                    ? String.format("LOCKED  %.0f in  %.0f deg  %.2f", aimbot.range, aimbot.bearingDeg, aimbot.confidence)
                    : aimbot.coasting ? "COLLECTING" : "SEARCHING";
            telemetry.addData("aimbot", "%s%s", state, assisting ? "" : "  (driver)");
            telemetry.addData("vision", "%.0f ms", vision.inferenceMs());
        } else {
            telemetry.addData("aimbot", "OFF  (dpad right)  %s", visionStatus);
        }
        telemetry.addData("drive", "%s%s", fieldCentric ? "FIELD" : "ROBOT", slowMode ? "  SLOW" : "");
        telemetry.addData("heading", "%.1f deg", Math.toDegrees(heading));
        telemetry.addData("wheels", "FL %.2f  FR %.2f  BL %.2f  BR %.2f", flPower, frPower, blPower, brPower);
        telemetry.addData("loop", "%.1f ms   battery %.2f V", loopMs, voltage);
        telemetry.update();
    }
}
