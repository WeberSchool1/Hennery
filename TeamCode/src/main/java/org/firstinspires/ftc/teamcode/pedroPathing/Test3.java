package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Config
@TeleOp(name="Test3", group="TeleOp")
public class Test3 extends LinearOpMode {

    // ----------------------
    // Hardware
    // ----------------------
    public static DcMotor frontLeft, frontRight, backLeft, backRight;
    public static DcMotor shooterMotor, frontIntake;
    public static Servo turretServo, turretHood;
    public static IMU imu;
    public static Limelight3A limelight;

    // ----------------------
    // Configurable parameters (tune in Dashboard)
    // ----------------------
    public static double DRIVE_SCALE = 0.8;

    public static double SHOOTER_MIN_POWER = 0.40;
    public static double SHOOTER_MAX_POWER = 0.72;
    public static double SHOOTER_DISTANCE_SCALE = 0.015;
    public static double SHOOTER_POWER_TO_TICKS_PER_SEC = 6000.0;
    public static double UP_TO_SPEED_TOLERANCE = 0.12;

    public static double SERVO_MIN_ANGLE_DEG = -45.0;
    public static double SERVO_MAX_ANGLE_DEG = 45.0;
    public static double SERVO_MIN_POS = 0.0;
    public static double SERVO_MAX_POS = 1.0;
    public static double TURRET_KP = 0.08;
    public static double TURRET_ALIGNMENT_SERVO_TOL = 0.02; // 0..1 servo pos fraction

    public static double HOOD_MIN_POS = 0.12;
    public static double HOOD_MAX_POS = 0.5;
    public static double HOOD_DISTANCE_SCALE = 0.005;

    public static double MANUAL_SERVO_STEP = 0.01;

    // ----------------------
    // Runtime variables
    // ----------------------
    private int lastShooterTicks = 0;
    private long lastTimeMs = 0;
    private double currentVelocityTicksPerSec = 0;
    private double targetVelocityTicksPerSec = 0;
    private boolean isUpToSpeed = false;
    private boolean manualOverrideActive = false;

    @Override
    public void runOpMode() throws InterruptedException {

        // ---- Hardware mapping ----
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");

        shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");
        frontIntake  = hardwareMap.get(DcMotor.class, "frontIntake");

        turretServo = hardwareMap.get(Servo.class, "turretTurnLeft");
        turretHood  = hardwareMap.get(Servo.class, "turretHood");

        imu = hardwareMap.get(IMU.class, "imu");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        FtcDashboard dashboard = FtcDashboard.getInstance();

        // ---- Motor setup ----
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        shooterMotor.setDirection(DcMotor.Direction.REVERSE);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lastShooterTicks = shooterMotor.getCurrentPosition();
        lastTimeMs = System.currentTimeMillis();

        limelight.pipelineSwitch(5);

        telemetry.addLine("Initialized. Press start.");
        telemetry.update();
        waitForStart();
        limelight.start();

        while (opModeIsActive()) {

            // -------------------------
            // Field-centric mecanum drive
            // -------------------------
            double driveX = -gamepad1.left_stick_x * DRIVE_SCALE;
            double driveY = -gamepad1.left_stick_y * DRIVE_SCALE;
            double turn   = -gamepad1.right_stick_x * DRIVE_SCALE;

            YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
            double headingRad = Math.toRadians(angles.getYaw());

            double tmpX = driveX * Math.cos(-headingRad) - driveY * Math.sin(-headingRad);
            double tmpY = driveX * Math.sin(-headingRad) + driveY * Math.cos(-headingRad);
            driveX = tmpX;
            driveY = tmpY;

            double fl = driveY + driveX + turn;
            double fr = driveY - driveX - turn;
            double bl = driveY - driveX + turn;
            double br = driveY + driveX - turn;

            double max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)), Math.max(Math.abs(bl), Math.abs(br)));
            if (max > 1.0) {
                fl /= max; fr /= max; bl /= max; br /= max;
            }

            frontLeft.setPower(fl);
            frontRight.setPower(fr);
            backLeft.setPower(bl);
            backRight.setPower(br);

            // -------------------------
            // Limelight info
            // -------------------------
            LLResult ll = limelight.getLatestResult();
            boolean targetVisible = (ll != null && ll.isValid());
            double tx = targetVisible ? ll.getTx() : 0.0;
            double ty = targetVisible ? ll.getTy() : 0.0;

            // -------------------------
            // Turret auto-align + manual D-pad override
            // -------------------------
            double desiredServoPos = turretServo.getPosition();
            if (targetVisible && gamepad1.x) {
                double logicalAngleDeg = -tx;
                logicalAngleDeg = Math.max(SERVO_MIN_ANGLE_DEG, Math.min(SERVO_MAX_ANGLE_DEG, logicalAngleDeg));
                double angRange = SERVO_MAX_ANGLE_DEG - SERVO_MIN_ANGLE_DEG;
                desiredServoPos = (logicalAngleDeg - SERVO_MIN_ANGLE_DEG) / angRange;
                desiredServoPos = Math.max(SERVO_MIN_POS, Math.min(SERVO_MAX_POS, desiredServoPos));
            }

            double currentServoPos = turretServo.getPosition();
            // manual override
            if (gamepad1.dpad_left) {
                manualOverrideActive = true;
                currentServoPos -= MANUAL_SERVO_STEP;
            } else if (gamepad1.dpad_right) {
                manualOverrideActive = true;
                currentServoPos += MANUAL_SERVO_STEP;
            } else {
                manualOverrideActive = false;
            }

            double nextServoPos;
            if (manualOverrideActive) {
                nextServoPos = currentServoPos;
            } else if (gamepad1.x && targetVisible) {
                double servoDelta = desiredServoPos - currentServoPos;
                nextServoPos = currentServoPos + servoDelta * TURRET_KP;
            } else {
                nextServoPos = currentServoPos;
            }

            nextServoPos = Math.max(SERVO_MIN_POS, Math.min(SERVO_MAX_POS, nextServoPos));
            turretServo.setPosition(nextServoPos);
            double turretServoError = desiredServoPos - turretServo.getPosition();
            boolean aligned = targetVisible && Math.abs(turretServoError) <= TURRET_ALIGNMENT_SERVO_TOL;

            // -------------------------
            // Hood auto-adjust
            // -------------------------
            boolean shooting = gamepad1.x || gamepad1.y || gamepad1.b;
            if (shooting && targetVisible) {
                double hoodDesired = HOOD_MIN_POS + (-ty * HOOD_DISTANCE_SCALE);
                hoodDesired = Math.max(HOOD_MIN_POS, Math.min(HOOD_MAX_POS, hoodDesired));
                turretHood.setPosition(hoodDesired);
            }

            // -------------------------
            // Shooter power based on distance
            // -------------------------
            double computedShooterPower = SHOOTER_MIN_POWER;
            if (targetVisible) {
                computedShooterPower = SHOOTER_MIN_POWER + (-ty * SHOOTER_DISTANCE_SCALE);
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

            targetVelocityTicksPerSec = computedShooterPower * SHOOTER_POWER_TO_TICKS_PER_SEC;
            isUpToSpeed = (Math.abs(currentVelocityTicksPerSec - targetVelocityTicksPerSec)
                    <= UP_TO_SPEED_TOLERANCE * Math.max(1.0, targetVelocityTicksPerSec));

            if (gamepad1.x) {
                shooterMotor.setPower(targetVisible ? computedShooterPower : 0.0);
            } else if (gamepad1.y) {
                shooterMotor.setPower(SHOOTER_MAX_POWER);
            } else if (gamepad1.b) {
                shooterMotor.setPower(SHOOTER_MIN_POWER);
            } else {
                shooterMotor.setPower(0.0);
            }

            // -------------------------
            // Intake control
            // -------------------------
            frontIntake.setPower(gamepad1.right_trigger - gamepad1.left_trigger);

            // -------------------------
            // Dashboard telemetry
            // -------------------------
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Target Visible", targetVisible);
            packet.put("tx", tx);
            packet.put("ty", ty);
            packet.put("Turret Servo", turretServo.getPosition());
            packet.put("Aligned?", aligned);
            packet.put("Turret Error", turretServoError);
            packet.put("Shooter Power", shooterMotor.getPower());
            packet.put("CurrentVel", currentVelocityTicksPerSec);
            packet.put("TargetVel", targetVelocityTicksPerSec);
            packet.put("UpToSpeed?", isUpToSpeed);
            packet.put("Manual Override?", manualOverrideActive);
            dashboard.sendTelemetryPacket(packet);
        }
    }
}
