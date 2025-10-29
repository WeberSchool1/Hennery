package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@TeleOp(name = "IntegratedTeleOp", group = "TeleOp")
public class IntegratedTeleOp extends LinearOpMode {

    // ===== Motors & Servos =====
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor shooterMotor;
    private DcMotor frontIntake;

    private Servo turretServo; // turretTurnRight
    private Servo turretHood;  // hood servo

    // Limelight
    private Limelight3A limelight;

    // IMU
    private IMU imu;

    // ===== Shooter & PID constants =====
    private static final double SHOOTER_MIN_POWER = 0.4; // short distance
    private static final double SHOOTER_MAX_POWER = 0.7; // long distance
    private static final double SHOOTER_DISTANCE_SCALE = 0.015; // tune for robot

    private static final double TURRET_KP = 0.02; // proportional for smooth PID
    private static final double TURRET_KI = 0.0;
    private static final double TURRET_KD = 0.0;

    private static final double HOOD_MIN_POS = 0.0;
    private static final double HOOD_MAX_POS = 1.0;

    private static final double UP_TO_SPEED_THRESHOLD = 0.1; // 10% tolerance

    // ===== Runtime variables =====
    private int lastShooterTicks = 0;
    private long lastTime = 0;
    private double currentVelocity = 0; // ticks/sec
    private double targetVelocity = 0;
    private boolean upToSpeed = false;

    @Override
    public void runOpMode() throws InterruptedException {

        // ===== Hardware mapping =====
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");

        shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");
        frontIntake  = hardwareMap.get(DcMotor.class, "frontIntake");

        turretServo  = hardwareMap.get(Servo.class, "turretTurnRight");
        turretHood   = hardwareMap.get(Servo.class, "turretHood");

        imu = hardwareMap.get(IMU.class, "imu");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // ===== Motor setup =====
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

        // Initialize encoder tracking
        lastShooterTicks = shooterMotor.getCurrentPosition();
        lastTime = System.currentTimeMillis();

        // Limelight pipeline
        limelight.pipelineSwitch(5);

        telemetry.addLine("Initialized. Waiting for start...");
        telemetry.update();
        waitForStart();

        limelight.start();

        while (opModeIsActive()) {

            // ===== Field-centric mecanum drive =====
            double driveX = -gamepad1.left_stick_x;
            double driveY = -gamepad1.left_stick_y;
            double turn   = -gamepad1.right_stick_x;

            YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
            double heading = Math.toRadians(angles.getYaw());

            double tempX = driveX * Math.cos(heading) - driveY * Math.sin(heading);
            double tempY = driveX * Math.sin(heading) + driveY * Math.cos(heading);

            driveX = tempX;
            driveY = tempY;

            double fl = driveY + driveX + turn;
            double fr = driveY - driveX - turn;
            double bl = driveY - driveX + turn;
            double br = driveY + driveX - turn;

            double max = Math.max(Math.abs(fl), Math.max(Math.abs(fr),
                        Math.max(Math.abs(bl), Math.abs(br))));
            if (max > 1.0) {
                fl /= max;
                fr /= max;
                bl /= max;
                br /= max;
            }

            frontLeft.setPower(fl);
            frontRight.setPower(fr);
            backLeft.setPower(bl);
            backRight.setPower(br);

            // ===== Limelight targeting =====
            LLResult result = limelight.getLatestResult();
            boolean targetVisible = (result != null && result.isValid());
            double tx = targetVisible ? result.getTx() : 0;
            double ty = targetVisible ? result.getTy() : 0;

            // ===== Turret PID auto-align =====
            double turretPower = 0;
            if (targetVisible) {
                turretPower = tx * TURRET_KP;
                // Clamp to safe range
                turretPower = Math.max(-1.0, Math.min(1.0, turretPower));
                turretServo.setPosition(turretServo.getPosition() + turretPower);
            }

            boolean aligned = Math.abs(tx) < 1.5; // degrees tolerance

            // ===== Shooter power based on distance (linear) =====
            double shooterPower = 0;
            if (targetVisible) {
                shooterPower = SHOOTER_MIN_POWER + (-ty + 10) * SHOOTER_DISTANCE_SCALE;
                shooterPower = Math.min(SHOOTER_MAX_POWER, Math.max(SHOOTER_MIN_POWER, shooterPower));
            }

            shooterMotor.setPower(0); // default off

            // ===== Shooter encoder velocity =====
            int currentTicks = shooterMotor.getCurrentPosition();
            long currentTime = System.currentTimeMillis();
            long deltaTime = currentTime - lastTime;

            if (deltaTime > 0) {
                currentVelocity = ((double)(currentTicks - lastShooterTicks)) / deltaTime * 1000; // ticks/sec
            }
            lastShooterTicks = currentTicks;
            lastTime = currentTime;

            targetVelocity = shooterPower * 6000; // GoBilda 6000 RPM scale
            upToSpeed = Math.abs(currentVelocity - targetVelocity) < (UP_TO_SPEED_THRESHOLD * targetVelocity);

            // ===== Hood auto-adjust =====
            if (targetVisible) {
                double hoodPos = HOOD_MIN_POS + (-ty + 10) * 0.005; // linear mapping
                hoodPos = Math.min(HOOD_MAX_POS, Math.max(HOOD_MIN_POS, hoodPos));
                turretHood.setPosition(hoodPos);
            }

            // ===== Shooting buttons =====
            // X: shoot at distance from Limelight
            if (gamepad1.x && targetVisible && aligned && upToSpeed) {
                shooterMotor.setPower(shooterPower);
            }

            // Y: shoot at max distance preset
            if (gamepad1.y) {
                shooterMotor.setPower(SHOOTER_MAX_POWER);
                turretHood.setPosition(HOOD_MAX_POS);
            }

            // B: shoot at min distance preset
            if (gamepad1.b) {
                shooterMotor.setPower(SHOOTER_MIN_POWER);
                turretHood.setPosition(HOOD_MIN_POS);
            }

            // ===== Intake control =====
            frontIntake.setPower(gamepad1.right_trigger - gamepad1.left_trigger);

            // ===== Telemetry =====
            telemetry.addData("Target Visible", targetVisible);
            telemetry.addData("TX", tx);
            telemetry.addData("TY", ty);
            telemetry.addData("Aligned", aligned);
            telemetry.addData("Shooter Power", shooterPower);
            telemetry.addData("Shooter Velocity", currentVelocity);
            telemetry.addData("Target Velocity", targetVelocity);
            telemetry.addData("Up to Speed?", upToSpeed);
            telemetry.addData("Turret Servo Pos", turretServo.getPosition());
            telemetry.addData("Hood Servo Pos",  turretHood.getPosition());
            telemetry.update();
        }
    }
}
