package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.List;

@TeleOp(name = "Priority", group = "Competition")
public class Priority extends OpMode {

    private static final double DEADZONE = 0.05;
    private static final double SLOW_SCALE = 0.35;
    private static final double STRAFE_FIX = 1.1;
    private static final double SLEW_RATE = 6.0;

    private static final double TICKS_PER_REV = 28.0;
    private static final double RPM_NEAR = 2400;
    private static final double RPM_MID = 2800;
    private static final double RPM_FAR = 3200;
    private static final double RPM_TRIM_STEP = 50;
    private static final double RPM_MIN = 1500;
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

    private static final double VOLTAGE_PERIOD = 0.5;
    private static final double CURRENT_PERIOD = 0.1;

    private enum Shooter { IDLE, SPINUP, READY, FEED, RECOVER }

    private DcMotor fl, fr, bl, br;
    private DcMotorEx intake, launcher;
    private CRServo leftIntakeServo, rightIntakeServo, feedServo;
    private VoltageSensor battery;
    private IMU imu;
    private List<LynxModule> hubs;

    private Shooter shooter = Shooter.IDLE;
    private double targetRPM = RPM_MID;
    private String preset = "MID";
    private double rpm;
    private double feedStartRPM;
    private int shots;
    private boolean announcedReady;

    private boolean slowMode;
    private boolean fieldCentric;
    private double heading;
    private double cmdForward, cmdStrafe;
    private double flPower, frPower, blPower, brPower;

    private double voltage = NOMINAL_VOLTAGE;
    private double intakeAmps;
    private boolean unjamming;

    private boolean lastSpin, lastFieldCentric, lastResetHeading;
    private boolean lastPresetNear, lastPresetMid, lastPresetFar, lastTrimUp, lastTrimDown;

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

        voltage = battery.getVoltage();
        applyLauncherGains();

        feedServo.setPower(0);
        leftIntakeServo.setPower(0);
        rightIntakeServo.setPower(0);

        telemetry.setMsTransmissionInterval(50);
        telemetry.addData("Priority", "initialized");
        telemetry.addData("imu", imu != null ? "found" : "missing, field centric disabled");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        telemetry.addData("Priority", "waiting for start");
        telemetry.addData("battery", "%.2f V", battery.getVoltage());
        telemetry.addData("imu", imu != null ? "found" : "missing");
        telemetry.addData("preset", "%s @ %.0f rpm", preset, targetRPM);
        telemetry.update();
    }

    @Override
    public void start() {
        shooter = Shooter.IDLE;
        shots = 0;
        loopTimer.reset();
        voltageTimer.reset();
        currentTimer.reset();
        if (imu != null) imu.resetYaw();
    }

    @Override
    public void loop() {
        for (LynxModule hub : hubs) hub.clearBulkCache();

        double dt = loopTimer.seconds();
        loopTimer.reset();
        loopMs = dt * 1000.0;

        readSensors();
        drive(dt);
        runIntake();
        runShooter();
        safetyStop();
        showTelemetry();
    }

    @Override
    public void stop() {
        haltEverything();
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
        }
        lastResetHeading = resetPressed;

        slowMode = gamepad1.left_bumper;

        double forward = shape(-gamepad1.left_stick_y);
        double strafe = shape(gamepad1.left_stick_x);
        double turn = shape(gamepad1.right_stick_x);

        if (fieldCentric) {
            double cos = Math.cos(-heading);
            double sin = Math.sin(-heading);
            double rotStrafe = strafe * cos - forward * sin;
            double rotForward = strafe * sin + forward * cos;
            strafe = rotStrafe;
            forward = rotForward;
        }

        strafe *= STRAFE_FIX;

        if (slowMode) {
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
        boolean mid = gamepad1.dpad_left || gamepad2.dpad_left;
        boolean far = gamepad1.dpad_up || gamepad2.dpad_up;
        boolean trimUp = gamepad2.right_bumper;
        boolean trimDown = gamepad2.left_bumper;

        if (near && !lastPresetNear) selectPreset("NEAR", RPM_NEAR);
        if (mid && !lastPresetMid) selectPreset("MID", RPM_MID);
        if (far && !lastPresetFar) selectPreset("FAR", RPM_FAR);
        if (trimUp && !lastTrimUp) setTarget(targetRPM + RPM_TRIM_STEP);
        if (trimDown && !lastTrimDown) setTarget(targetRPM - RPM_TRIM_STEP);

        lastPresetNear = near;
        lastPresetMid = mid;
        lastPresetFar = far;
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
                } else if (fire) {
                    feedStartRPM = rpm;
                    shooterTimer.reset();
                    shooter = Shooter.FEED;
                }
                break;

            case FEED:
                feedServo.setPower(FEED_POWER);
                boolean dipped = feedStartRPM - rpm > SHOT_DIP_RPM;
                if (dipped || shooterTimer.seconds() > FEED_MAX_TIME) {
                    if (dipped) shots++;
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

    private void selectPreset(String name, double rpmTarget) {
        preset = name;
        setTarget(rpmTarget);
    }

    private void setTarget(double rpmTarget) {
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
        telemetry.addData("loop", "%.1f ms", loopMs);
        telemetry.addData("battery", "%.2f V", voltage);
        telemetry.addLine();
        telemetry.addData("shooter", shooter);
        telemetry.addData("preset", "%s  target %.0f  actual %.0f", preset, targetRPM, rpm);
        telemetry.addData("shots", shots);
        telemetry.addLine();
        telemetry.addData("drive", "%s%s", fieldCentric ? "FIELD" : "ROBOT", slowMode ? "  SLOW" : "");
        telemetry.addData("heading", "%.1f deg", Math.toDegrees(heading));
        telemetry.addData("wheels", "FL %.2f  FR %.2f  BL %.2f  BR %.2f", flPower, frPower, blPower, brPower);
        telemetry.addLine();
        telemetry.addData("intake", "%s  %.2f A", unjamming ? "UNJAM" : "OK", intakeAmps);
        telemetry.update();
    }
}
