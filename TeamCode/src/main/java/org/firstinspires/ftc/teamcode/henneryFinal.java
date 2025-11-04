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
    private DcMotor shooterMotor, frontIntake, backIntake, turretSpin;
    private Servo turretServo, turretHood;
    private Limelight3A limelight;

    private Servo redLed;

    private static  double SERVO_MIN_POS = 0.0;
    private static  double SERVO_MAX_POS = 1.0;

    public static double MANUAL_SERVO_STEP = 0.01;

    @Override
    public void runOpMode() throws InterruptedException {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");
        frontIntake = hardwareMap.get(DcMotor.class, "frontIntake");
        turretSpin =  hardwareMap.get(DcMotor.class, "turretOne");

        turretServo = hardwareMap.get(Servo.class, "turretTurn");
        turretHood = hardwareMap.get(Servo.class, "turretHood");

        backIntake = hardwareMap.get(DcMotor.class, "backIntake");

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

            double fl = driveY - driveX + turn;
            double fr = driveY + driveX - turn;
            double bl = driveY + driveX + turn;
            double br = driveY - driveX - turn;

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

            if (gamepad1.y) {
                backIntake.setPower(.75);}
            else {
                backIntake.setPower(0);
            }


            if (gamepad1.a) {
                shooterMotor.setPower(.75);
            } else if (gamepad1.b) {
                shooterMotor.setPower(.45);
            } else {
                shooterMotor.setPower(0);
            }

            if (gamepad1.dpad_left) {
                turretSpin.setPower(1);}
            else if (gamepad1.dpad_right){
                    turretSpin.setPower(-1);}
            else {
                turretSpin.setPower(0);
            }


            double intakePower = gamepad1.right_trigger - gamepad1.left_trigger;
            // optional scale if you want to reduce top speed: intakePower *= 0.9;
            frontIntake.setPower(intakePower);

            if(gamepad1.right_bumper){
                turretHood.setPosition(.8);
            }else if (gamepad1.left_bumper){
                turretHood.setPosition(.45);
            }

            if (targetVisible) { // Light up the LED if the X button is pressedm
                redLed.setPosition(1);
            } else {
                redLed.setPosition(0);
            }


        }


    }
}


