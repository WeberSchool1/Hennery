package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "henneryFinalAutoTurret", group = "TeleOp")
public class henneryFinalAutoTurret extends LinearOpMode {
    private static final double DRIVE_SCALE = 0.85;
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor shooterMotor, frontIntake, backIntake, turretSpin;
    private Servo turretServo, turretHood, redLed;
    private Limelight3A limelight;

    // Simple proportional control for turret auto-align
    private final double kP_TURRET = 0.02; // tune as needed
    private final double deadbandDeg = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
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

            // --- Shooter controls ---
            if (gamepad1.a) shooterMotor.setPower(0.75);
            else if (gamepad1.b) shooterMotor.setPower(0.45);
            else shooterMotor.setPower(0);

            // --- Intake controls ---
            double intakePower = gamepad1.right_trigger - gamepad1.left_trigger;
            frontIntake.setPower(intakePower);

            if (gamepad1.y) backIntake.setPower(0.75);
            else backIntake.setPower(0);

            // --- Turret hood ---
            if(gamepad1.right_bumper) turretHood.setPosition(0.8);
            else if (gamepad1.left_bumper) turretHood.setPosition(0.45);

            // --- LED feedback ---
            if (targetVisible) redLed.setPosition(1);
            else redLed.setPosition(0);

            telemetry.addData("tx", tx);
            telemetry.addData("Target Visible", targetVisible);
            telemetry.update();
        }
    }

    private double clamp(double val, double min, double max) {
        if (val < min) return min;
        if (val > max) return max;
        return val;
    }
}
