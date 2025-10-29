package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Test1", group = "TeleOp")
public class Test1 extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor shooterMotor, frontIntake;
    private Servo turretServo, turretHood;
    private Limelight3A limelight;

    // Drive tuning
    private static final double DRIVE_SCALE = 0.8;

    // Turret tuning
    private static final double SERVO_MIN_POS = 0.0;
    private static final double SERVO_MAX_POS = 1.0;
    private static final double SERVO_MIN_ANGLE_DEG = -45;
    private static final double SERVO_MAX_ANGLE_DEG = 45;
    private static final double TURRET_KP = 0.08;
    private static final double TURRET_ALIGNMENT_DEG_TOL = 2.0;

    // Shooter tuning
    private static final double SHOOTER_MIN_POWER = 0.4;
    private static final double SHOOTER_MAX_POWER = .87;
    private static final double UP_TO_SPEED_TOLERANCE = 0.05;

    private static final double SHOOTER_DISTANCE_SCALE = 0.005;

    // Hood tuning
    private static final double HOOD_MIN_POS = 0.12;
    private static final double HOOD_MAX_POS = 0.5;
    private static final double HOOD_DISTANCE_SCALE = 0.005;

    // Limelight mounting values
    private static final double CAMERA_HEIGHT = 13.3; // inches
    private static final double TARGET_HEIGHT = 36.0; // inches
    private static final double CAMERA_ANGLE_DEG = 19.0; // mounting angle

    // Shooter velocity tracking
    private int lastShooterTicks = 0;
    private long lastTimeMs = 0;
    private double currentVelocityTicksPerSec = 0.0;
    private double targetVelocityTicksPerSec = 0.0;
    private boolean isUpToSpeed = false;

    @Override
    public void runOpMode() {

        // Hardware map
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");
        shooterMotor.setDirection(DcMotor.Direction.REVERSE);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontIntake = hardwareMap.get(DcMotor.class, "frontIntake");

        turretServo = hardwareMap.get(Servo.class, "turretTurnRight");
        turretHood = hardwareMap.get(Servo.class, "turretHood");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(5);

        telemetry.addLine("Initialized. Waiting for start...");
        telemetry.update();
        waitForStart();
        limelight.start();

        while (opModeIsActive()) {

            // -------------------------
            // Field-centric mecanum drive
            // -------------------------
            double y = -gamepad1.left_stick_y * DRIVE_SCALE;
            double x = gamepad1.left_stick_x * DRIVE_SCALE;
            double rot = gamepad1.right_stick_x * DRIVE_SCALE;

            double fl = y + x + rot;
            double fr = y - x - rot;
            double bl = y - x + rot;
            double br = y + x - rot;

            double max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)), Math.max(Math.abs(bl), Math.abs(br)));
            if (max > 1.0) { fl /= max; fr /= max; bl /= max; br /= max; }

            frontLeft.setPower(fl);
            frontRight.setPower(fr);
            backLeft.setPower(bl);
            backRight.setPower(br);

            // -------------------------
            // Limelight target info
            // -------------------------
            LLResult ll = limelight.getLatestResult();
            boolean targetVisible = (ll != null && ll.isValid());
            double tx = targetVisible ? ll.getTx() : 0.0;
            double ty = targetVisible ? ll.getTy() : 0.0;

            // -------------------------
            // Calculate distance to AprilTag
            // -------------------------
            double distance = 0.0;
            if (targetVisible) {
                distance = (TARGET_HEIGHT - CAMERA_HEIGHT) /
                        Math.tan(Math.toRadians(CAMERA_ANGLE_DEG + ty));
            }

            // -------------------------
            // Turret auto-align
            // -------------------------
            double desiredServoPos = 0.5; // center by default
            if (targetVisible) {
                double logicalAngle = -tx;
                logicalAngle = Math.max(SERVO_MIN_ANGLE_DEG, Math.min(SERVO_MAX_ANGLE_DEG, logicalAngle));
                desiredServoPos = (logicalAngle - SERVO_MIN_ANGLE_DEG) / (SERVO_MAX_ANGLE_DEG - SERVO_MIN_ANGLE_DEG);
                desiredServoPos = Math.max(SERVO_MIN_POS, Math.min(SERVO_MAX_POS, desiredServoPos));
            }

            double currentServoPos = turretServo.getPosition();
            double nextServoPos = currentServoPos + (desiredServoPos - currentServoPos) * TURRET_KP;
            nextServoPos = Math.max(SERVO_MIN_POS, Math.min(SERVO_MAX_POS, nextServoPos));
            turretServo.setPosition(nextServoPos);

            double turretError = desiredServoPos - turretServo.getPosition();
            boolean aligned = targetVisible && Math.abs(turretError) <= TURRET_ALIGNMENT_DEG_TOL;

            // -------------------------
            // Hood auto-adjust based on distance
            // -------------------------
            boolean shooting = gamepad1.x || gamepad1.y || gamepad1.b;
            if (shooting) {
                double hoodPos = HOOD_MIN_POS + distance * HOOD_DISTANCE_SCALE;
                hoodPos = Math.max(HOOD_MIN_POS, Math.min(HOOD_MAX_POS, hoodPos));
                turretHood.setPosition(hoodPos);
            }

            // -------------------------
            // Shooter power based on distance
            // -------------------------
            double computedShooterPower = SHOOTER_MIN_POWER;
            if (targetVisible) {
                computedShooterPower = SHOOTER_MIN_POWER + distance * SHOOTER_DISTANCE_SCALE;
                computedShooterPower = Math.max(SHOOTER_MIN_POWER, Math.min(SHOOTER_MAX_POWER, computedShooterPower));
            }

            int currentTicks = shooterMotor.getCurrentPosition();
            long nowMs = System.currentTimeMillis();
            long dt = nowMs - lastTimeMs;
            if (dt > 0) {
                currentVelocityTicksPerSec = ((double)(currentTicks - lastShooterTicks)) / dt * 1000.0;
            }
            lastShooterTicks = currentTicks;
            lastTimeMs = nowMs;

            targetVelocityTicksPerSec = computedShooterPower * 6000; // adjust if motor RPM differs
            isUpToSpeed = Math.abs(currentVelocityTicksPerSec - targetVelocityTicksPerSec)
                    <= UP_TO_SPEED_TOLERANCE * Math.max(1.0, targetVelocityTicksPerSec);

            // -------------------------
            // Shooter control
            // -------------------------
            if (gamepad1.x) shooterMotor.setPower(computedShooterPower);
            else if (gamepad1.y) shooterMotor.setPower(SHOOTER_MAX_POWER);
            else if (gamepad1.b) shooterMotor.setPower(SHOOTER_MIN_POWER);
            else shooterMotor.setPower(0.0);

            // -------------------------
            // Intake control
            // -------------------------
            double intakePower = gamepad1.right_trigger - gamepad1.left_trigger;
            frontIntake.setPower(intakePower);

            // -------------------------
            // Telemetry
            // -------------------------
            telemetry.addData("Target Visible", targetVisible);
            telemetry.addData("tx", "%.2f", tx);
            telemetry.addData("ty", "%.2f", ty);
            telemetry.addData("Distance (in)", "%.2f", distance);
            telemetry.addData("Turret Pos", "%.3f", turretServo.getPosition());
            telemetry.addData("Aligned?", aligned);
            telemetry.addData("Turret Error", "%.3f", turretError);
            telemetry.addData("Shooter Power", "%.3f", shooterMotor.getPower());
            telemetry.addData("Current Vel (ticks/s)", "%.1f", currentVelocityTicksPerSec);
            telemetry.addData("Target Vel (ticks/s)", "%.1f", targetVelocityTicksPerSec);
            telemetry.addData("UpToSpeed?", isUpToSpeed);
            telemetry.update();
        }
    }
}
