package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "henneryRed", group = "TeleOp")
public class henneryRed extends LinearOpMode {
    private static final double DRIVE_SCALE = 0.85;

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor shooterMotor, frontIntake, backIntake, turretSpin;
    private Servo turretHood, rightLed, leftLed;
    private TouchSensor touchSensor;
    private Limelight3A limelight;

    // Turret auto-align constants
    private final double kP_TURRET = 0.02;
    private final double deadbandDeg = 0.5;

    // Shooter auto-speed constants
    private final double CAMERA_HEIGHT = 12.54; // inches
    private final double TARGET_HEIGHT = 24.0;  // example: hub height
    private final double CAMERA_ANGLE = 19.0;   // degrees
    private final double MIN_DISTANCE = 20.0;   // inches
    private final double MAX_DISTANCE = 140.0;  // inches
    private final double MIN_POWER = 0.45;
    private final double MAX_POWER = 0.78;

    @Override
    public void runOpMode() throws InterruptedException {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry telemetry = dashboard.getTelemetry();


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

        touchSensor = hardwareMap.get(TouchSensor.class, "touchsensor");
        turretHood = hardwareMap.get(Servo.class, "turretHood");
        leftLed = hardwareMap.get(Servo.class, "LEDLeft");
        rightLed = hardwareMap.get(Servo.class, "LEDRight");


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
            double targetShooterPower = 0.0; // initialize target power

            if (gamepad1.y) {
                double distance = (TARGET_HEIGHT - CAMERA_HEIGHT) /
                        Math.tan(Math.toRadians(CAMERA_ANGLE + ty));
                distance = Math.max(MIN_DISTANCE, Math.min(MAX_DISTANCE, distance));
                targetShooterPower = MIN_POWER + (distance - MIN_DISTANCE) / (MAX_DISTANCE - MIN_DISTANCE) * (MAX_POWER - MIN_POWER);
                shooterMotor.setPower(targetShooterPower);
            } else {
                shooterMotor.setPower(0.0);
                targetShooterPower = 0.0;
            }

// --- Intake controls ---
            double intakePower = gamepad1.right_trigger - gamepad1.left_trigger;
            frontIntake.setPower(intakePower);

            if (gamepad1.right_bumper) {
                backIntake.setPower(.9);
            } else {
                backIntake.setPower(0);}


// --- Turret hood ---
            if (gamepad1.dpad_right) turretHood.setPosition(0.8);
            else if (gamepad1.dpad_left) turretHood.setPosition(0.45);

// --- LED feedback ---
            if (targetVisible){
            leftLed.setPosition(.611);}
            else { leftLed.setPosition(0);}

            if (touchSensor.isPressed()) {
                rightLed.setPosition(.277);
                telemetry.addData("Touch Sensor", "Is Pressed");
            } else {
                telemetry.addData("Touch Sensor", "Is Not Pressed");
            }



// --- Telemetry ---
            TelemetryPacket packet = new TelemetryPacket();

            telemetry.addData("Target Visible", targetVisible);
            telemetry.addData("tx", tx);
            telemetry.addData("ty", ty);
            telemetry.addData("Shooter Power", shooterMotor.getPower());
            telemetry.addData("Target Shooter Power", targetShooterPower);

// Shooter up-to-speed check (tolerance 0.02)
            boolean shooterAtSpeed = Math.abs(shooterMotor.getPower() - targetShooterPower) < 0.02;
            telemetry.addData("Shooter At Speed", shooterAtSpeed);
            packet.put("Shooter At Speed", shooterAtSpeed);

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
