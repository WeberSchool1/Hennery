package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

@TeleOp(name = "henneryFinalAutoTurretShooter_AutoShoot", group = "TeleOp")
public class henneryFinalAutoTurretShooter_AutoShoot extends LinearOpMode {
    private static final double DRIVE_SCALE = 0.85;

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor shooterMotor, frontIntake, backIntake, turretSpin;
    private Servo turretServo, turretHood, redLed;
    private Limelight3A limelight;

    // Turret auto-align constants
    private final double kP_TURRET = 0.02;
    private final double deadbandDeg = 0.5;

    // Shooter auto-speed constants
    private final double CAMERA_HEIGHT = 12.54; // inches
    private final double TARGET_HEIGHT = 36.0;  // 3 ft to top of AprilTag
    private final double CAMERA_ANGLE = 70.0;   // tilted up 70 deg
    private final double MIN_DISTANCE = 20.0;   // inches
    private final double MAX_DISTANCE = 100.0;  // inches
    private final double MIN_POWER = 0.45;
    private final double MAX_POWER = 0.75;

    // Auto-shoot parameters
    private final double ALIGN_TOLERANCE = 1.0;   // degrees tx tolerance
    private final long SHOOTER_SPINUP_MS = 3000;   // wait before feed
    private final long FEED_DURATION_MS = 1000;    // feeder run time

    @Override
    public void runOpMode() throws InterruptedException {

        FtcDashboard dashboard = FtcDashboard.getInstance();

        // --- Hardware mapping ---
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");
        shooterMotor.setDirection(DcMotor.Direction.REVERSE);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontIntake = hardwareMap.get(DcMotor.class, "frontIntake");
        backIntake = hardwareMap.get(DcMotor.class, "backIntake");
        turretSpin = hardwareMap.get(DcMotor.class, "turretOne");

        turretServo = hardwareMap.get(Servo.class, "turretTurn");
        turretHood = hardwareMap.get(Servo.class, "turretHood");
        redLed = hardwareMap.get(Servo.class, "LEDLeft");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(5);

        telemetry.addLine("henneryFinal initialized. Press start.");
        telemetry.update();

        waitForStart();
        limelight.start();

        while (opModeIsActive()) {
            // --- Drive controls ---
            double driveY = -gamepad1.left_stick_y * DRIVE_SCALE;
            double driveX = gamepad1.left_stick_x * DRIVE_SCALE;
            double turn = gamepad1.right_stick_x * DRIVE_SCALE;

            double fl = driveY - driveX + turn;
            double fr = driveY + driveX - turn;
            double bl = driveY + driveX + turn;
            double br = driveY - driveX - turn;

            double max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                                  Math.max(Math.abs(bl), Math.abs(br)));
            if (max > 1.0) {
                fl /= max; fr /= max; bl /= max; br /= max;
            }

            frontLeft.setPower(fl);
            frontRight.setPower(fr);
            backLeft.setPower(bl);
            backRight.setPower(br);

            // --- Limelight read ---
            LLResult ll = limelight.getLatestResult();
            boolean targetVisible = (ll != null && ll.isValid());
            double tx = targetVisible ? ll.getTx() : 0.0;
            double ty = targetVisible ? ll.getTy() : 0.0;

            // --- Turret auto-align ---
            if (targetVisible) {
                if (Math.abs(tx) > deadbandDeg) {
                    double turretPower = kP_TURRET * tx;
                    turretPower = clamp(turretPower, -0.5, 0.5);
                    turretSpin.setPower(turretPower);
                } else {
                    turretSpin.setPower(0.0);
                }
            } else {
                turretSpin.setPower(0.0);
            }

            // --- Shooter auto-speed ---
            if (targetVisible && gamepad1.a) {
                double distance = (TARGET_HEIGHT - CAMERA_HEIGHT) /
                        Math.tan(Math.toRadians(CAMERA_ANGLE + ty));
                distance = Math.max(MIN_DISTANCE, Math.min(MAX_DISTANCE, distance));
                double shooterPower = MIN_POWER + (distance - MIN_DISTANCE) / (MAX_DISTANCE - MIN_DISTANCE) * (MAX_POWER - MIN_POWER);
                shooterMotor.setPower(shooterPower);
            } else if (!gamepad1.x) { // prevent auto-shoot override
                shooterMotor.setPower(0.0);
            }

            // --- Auto-Shoot Routine (gamepad1.x) ---
            if (gamepad1.x && targetVisible) {
                double distance = (TARGET_HEIGHT - CAMERA_HEIGHT) /
                        Math.tan(Math.toRadians(CAMERA_ANGLE + ty));
                distance = Math.max(MIN_DISTANCE, Math.min(MAX_DISTANCE, distance));
                double shooterPower = MIN_POWER + (distance - MIN_DISTANCE) / (MAX_DISTANCE - MIN_DISTANCE) * (MAX_POWER - MIN_POWER);

                // 1. Align turret until tx small
                while (opModeIsActive()) {
                    LLResult alignResult = limelight.getLatestResult();
                    if (alignResult == null || !alignResult.isValid()) break;
                    double txNow = alignResult.getTx();
                    if (Math.abs(txNow) < ALIGN_TOLERANCE) {
                        turretSpin.setPower(0);
                        break;
                    }
                    double turretPower = kP_TURRET * txNow;
                    turretPower = clamp(turretPower, -0.4, 0.4);
                    turretSpin.setPower(turretPower);
                    telemetry.addData("Aligning...", txNow);
                    telemetry.update();
                }

                // 2. Spin shooter up
                shooterMotor.setPower(shooterPower);
                sleep(SHOOTER_SPINUP_MS);

                // 3. Feed one shot
                backIntake.setPower(0.8);
                sleep(FEED_DURATION_MS);
                backIntake.setPower(0.0);

                // 4. Stop shooter after short delay
                sleep(2000);
                shooterMotor.setPower(0.0);
            }

            // --- Intake controls ---
            double intakePower = gamepad1.right_trigger - gamepad1.left_trigger;
            frontIntake.setPower(intakePower);

            if (gamepad1.y) backIntake.setPower(0.75);
            else if (!gamepad1.x) backIntake.setPower(0);

            // --- Turret hood ---
            if (gamepad1.right_bumper) turretHood.setPosition(0.8);
            else if (gamepad1.left_bumper) turretHood.setPosition(0.45);

            // --- LED feedback ---
            redLed.setPosition(targetVisible ? 1 : 0);

            // --- Telemetry ---
            TelemetryPacket packet = new TelemetryPacket();
            telemetry.addData("Target Visible", targetVisible);
            telemetry.addData("tx", tx);
            telemetry.addData("ty", ty);
            telemetry.addData("Shooter Power", shooterMotor.getPower());
            telemetry.update();
            dashboard.sendTelemetryPacket(packet);
        }
    }

    private double clamp(double val, double min, double max) {
        if (val < min) return min;
        if (val > max) return max;
        return val;
    }
}
