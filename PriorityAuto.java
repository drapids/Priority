package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Priority Auto", group = "Competition", preselectTeleOp = "Priority")
public class PriorityAuto extends LinearOpMode {

    private static final double TICKS_PER_REV = 28.0;
    private static final double RPM_HIVE = 2400;
    private static final double RPM_TOLERANCE = 80;
    private static final double SPINUP_TIMEOUT = 2.5;
    private static final double SHOT_DIP_RPM = 150;
    private static final double FEED_MAX_TIME = 0.5;
    private static final double BETWEEN_SHOTS = 0.35;
    private static final int PRELOADS = 3;

    private static final double LAUNCHER_P = 12;
    private static final double LAUNCHER_I = 3;
    private static final double LAUNCHER_D = 0;
    private static final double LAUNCHER_F = 14;
    private static final double NOMINAL_VOLTAGE = 12.5;

    private static final double LEAVE_POWER = 0.4;
    private static final double LEAVE_TIME = 1.2;

    private DcMotor fl, fr, bl, br;
    private DcMotorEx intake, launcher;
    private CRServo leftIntakeServo, rightIntakeServo, feedServo;
    private VoltageSensor battery;

    private int startDelay = 0;
    private boolean shoot = true;

    private final ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {
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
        rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);

        for (DcMotor m : new DcMotor[]{fl, fr, bl, br}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        boolean lastUp = false, lastDown = false, lastA = false;
        while (opModeInInit()) {
            if (gamepad1.dpad_up && !lastUp) startDelay = Math.min(startDelay + 1, 15);
            if (gamepad1.dpad_down && !lastDown) startDelay = Math.max(startDelay - 1, 0);
            if (gamepad1.a && !lastA) shoot = !shoot;
            lastUp = gamepad1.dpad_up;
            lastDown = gamepad1.dpad_down;
            lastA = gamepad1.a;

            telemetry.addData("Priority Auto", "ready");
            telemetry.addData("start delay", "%d s   (dpad up/down)", startDelay);
            telemetry.addData("shoot preloads", "%s   (A)", shoot ? "YES" : "NO, leave only");
            telemetry.addData("battery", "%.2f V", battery.getVoltage());
            telemetry.update();
        }

        if (isStopRequested()) return;

        sleep(startDelay * 1000L);

        if (shoot && opModeIsActive()) shootPreloads();

        drive(LEAVE_POWER);
        timer.reset();
        while (opModeIsActive() && timer.seconds() < LEAVE_TIME) {
            telemetry.addData("step", "leave");
            telemetry.update();
        }
        drive(0);

        telemetry.addData("step", "done");
        telemetry.update();
    }

    private void shootPreloads() {
        double comp = NOMINAL_VOLTAGE / Math.max(battery.getVoltage(), 9.0);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(LAUNCHER_P, LAUNCHER_I, LAUNCHER_D, LAUNCHER_F * comp));
        launcher.setVelocity(RPM_HIVE * TICKS_PER_REV / 60.0);

        int fired = 0;
        while (opModeIsActive() && fired < PRELOADS) {
            timer.reset();
            while (opModeIsActive() && Math.abs(rpm() - RPM_HIVE) > RPM_TOLERANCE
                    && timer.seconds() < SPINUP_TIMEOUT) {
                status("spin up", fired);
            }

            double startRPM = rpm();
            feedServo.setPower(1.0);
            setIntake(0.6);
            timer.reset();
            while (opModeIsActive() && startRPM - rpm() < SHOT_DIP_RPM
                    && timer.seconds() < FEED_MAX_TIME) {
                status("feed", fired);
            }
            feedServo.setPower(0);
            setIntake(0);
            fired++;

            timer.reset();
            while (opModeIsActive() && timer.seconds() < BETWEEN_SHOTS) {
                status("recover", fired);
            }
        }

        launcher.setVelocity(0);
        launcher.setPower(0);
    }

    private double rpm() {
        return launcher.getVelocity() * 60.0 / TICKS_PER_REV;
    }

    private void setIntake(double power) {
        intake.setPower(power);
        leftIntakeServo.setPower(power);
        rightIntakeServo.setPower(power);
    }

    private void drive(double power) {
        fl.setPower(power);
        fr.setPower(power);
        bl.setPower(power);
        br.setPower(power);
    }

    private void status(String step, int fired) {
        telemetry.addData("step", step);
        telemetry.addData("fired", "%d / %d", fired, PRELOADS);
        telemetry.addData("rpm", "%.0f / %.0f", rpm(), RPM_HIVE);
        telemetry.update();
    }
}
