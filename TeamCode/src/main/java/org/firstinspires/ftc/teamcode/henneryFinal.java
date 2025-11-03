package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.hardware.LED;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

@TeleOp(name = "henneryFinal", group = "TeleOp")
public class henneryFinal extends LinearOpMode {
    private static final double DRIVE_SCALE = 0.85;
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor shooterMotor, frontIntake;
    private Servo turretServo, turretHood;
    private Limelight3A limelight;

    private Servo redLed;

    @Override
    public void runOpMode() throws InterruptedException {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");
        frontIntake = hardwareMap.get(DcMotor.class, "frontIntake");

        turretServo = hardwareMap.get(Servo.class, "turretTurn");
        turretHood = hardwareMap.get(Servo.class, "turretHood");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        //shooter motor reveresed
        shooterMotor.setDirection(DcMotor.Direction.REVERSE);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        redLed = hardwareMap.get(Servo.class, "LEDLeft");

        //limelight
        limelight.pipelineSwitch(5);

        //telemetye check
        telemetry.addLine("henneryFinal initialized. Press start.");
        telemetry.update();
        waitForStart();

        limelight.start();

        while (opModeIsActive()) {

            double driveY = -gamepad1.left_stick_y * DRIVE_SCALE; // forward
            double driveX = gamepad1.left_stick_x * DRIVE_SCALE; // strafe
            double turn = gamepad1.right_stick_x * DRIVE_SCALE; // rotate

            double fl = driveY + driveX + turn;
            double fr = driveY - driveX - turn;
            double bl = driveY - driveX + turn;
            double br = driveY + driveX - turn;

            // normalize
            double max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)), Math.max(Math.abs(bl), Math.abs(br)));
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

            // -------------------------
            // Limelight read
            // -------------------------
            LLResult ll = limelight.getLatestResult();
            boolean targetVisible = (ll != null && ll.isValid());
            double tx = targetVisible ? ll.getTx() : 0.0;
            double ty = targetVisible ? ll.getTy() : 0.0;

            if (gamepad1.a) {
                shooterMotor.setPower(.75);
            } else if (gamepad1.b) {
                shooterMotor.setPower(.45);
            } else {
                shooterMotor.setPower(0);
            }

            if (gamepad1.dpad_left) {
                turretServo.setPosition(90);
            }
            if (gamepad1.dpad_right) {
                turretServo.setPosition(-90);
            }
            if (gamepad1.dpad_up) {
                turretServo.setPosition(.5);


            }

            if (gamepad1.x) { // Light up the LED if the X button is pressedm
                redLed.setPosition(1);
            } else {
                redLed.setPosition(0);
            }


        }


    }
}

