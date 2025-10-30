package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@TeleOp(name = "Test3", group = "TeleOp")
public class Test3 extends LinearOpMode {

    // -------------------------------
    // Tunable Variables (FTC Dashboard)
    // -------------------------------
    public static double SHOOTER_MIN_POWER = 0.40;
    public static double SHOOTER_MAX_POWER = 0.72;
    public static double SHOOTER_DISTANCE_SCALE = 0.015;

    public static double SERVO_TOTAL_ANGLE_DEG = 300.0;  // physical servo rotation
    public static double SERVO_MIN_ANGLE_DEG = -150.0;    // logical left limit
    public static double SERVO_MAX_ANGLE_DEG = 150.0;     // logical right limit
    public static double SERVO_MIN_POS = 0.0;
    public static double SERVO_MAX_POS = 1.0;
    public static double SERVO_CENTER_POS = 0.5;         // servo pos when turret faces forward
    public static double TURRET_KP = 0.08;
    public static double TURRET_ALIGNMENT_DEG_TOL = 2.0;
    public static double MANUAL_SERVO_STEP = 0.01;       // D-pad step

    public static double HOOD_FIXED_POS = 0.6;          // hood stays fixed

    // -------------------------------
    // Hardware
    // -------------------------------
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor shooterMotor, frontIntake;
    private Servo turretServo, turretHood;
    private Limelight3A limelight;
    private FtcDashboard dashboard;
    private Telemetry dashTelemetry;

    private boolean manualOverrideActive = false;

    @Override
    public void runOpMode() {

        // Map hardware
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");
        frontIntake = hardwareMap.get(DcMotor.class, "frontIntake");
        turretServo = hardwareMap.get(Servo.class, "turretTurnLeft");
        turretHood = hardwareMap.get(Servo.class, "turretHood");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterMotor.setDirection(DcMotor.Direction.REVERSE);
        frontIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // FTC Dashboard setup
        dashboard = FtcDashboard.getInstance();
        dashTelemetry = dashboard.getTelemetry();

        limelight.pipelineSwitch(5);

        telemetry.addLine("Initialized — waiting for start...");
        telemetry.update();

        waitForStart();
        limelight.start();

        // Main loop
        while (opModeIsActive()) {

            // -----------------------------
            // Basic Tank/Mecanum Drive (simple)
            // -----------------------------
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rot = gamepad1.right_stick_x;

            double fl = y + x + rot;
            double fr = y - x - rot;
            double bl = y - x + rot;
            double br = y + x - rot;

            double max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)), Math.max(Math.abs(bl), Math.abs(br)));
            if (max > 1.0) {
                fl /= max; fr /= max; bl /= max; br /= max;
            }

            frontLeft.setPower(fl);
            frontRight.setPower(fr);
            backLeft.setPower(bl);
            backRight.setPower(br);

            // -----------------------------
            // Limelight target tracking
            // -----------------------------
            LLResult ll = limelight.getLatestResult();
            boolean targetVisible = (ll != null && ll.isValid());
            double tx = targetVisible ? ll.getTx() : 0.0;
            double ty = targetVisible ? ll.getTy() : 0.0;

            // -----------------------------
            // Turret Control
            // -----------------------------
            double desiredServoPos = turretServo.getPosition();

            if (targetVisible) {
                double logicalAngleDeg = -tx;  // negative = left
                logicalAngleDeg = Math.max(SERVO_MIN_ANGLE_DEG, Math.min(SERVO_MAX_ANGLE_DEG, logicalAngleDeg));

                // Convert logical angle to servo position using center offset + scaling
                double normalizedAngle = logicalAngleDeg / (SERVO_TOTAL_ANGLE_DEG / 2.0); // scale to ±0.5 range
                desiredServoPos = SERVO_CENTER_POS + normalizedAngle;
                desiredServoPos = Math.max(SERVO_MIN_POS, Math.min(SERVO_MAX_POS, desiredServoPos));
            }

            double currentServoPos = turretServo.getPosition();

            // Manual override (D-pad)
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
                nextServoPos = turretServo.getPosition();
            }

            nextServoPos = Math.max(SERVO_MIN_POS, Math.min(SERVO_MAX_POS, nextServoPos));
            turretServo.setPosition(nextServoPos);

            boolean aligned = targetVisible && Math.abs(tx) <= TURRET_ALIGNMENT_DEG_TOL;

            // -----------------------------
            // Shooter Power Control
            // -----------------------------
            double shooterPower = 0.0;
            if (targetVisible) {
                shooterPower = SHOOTER_MIN_POWER + (-ty + 10.0) * SHOOTER_DISTANCE_SCALE;
                shooterPower = Math.max(SHOOTER_MIN_POWER, Math.min(SHOOTER_MAX_POWER, shooterPower));
            }

            if (gamepad1.x && targetVisible) {
                shooterMotor.setPower(shooterPower);
            } else if (gamepad1.y) {
                shooterMotor.setPower(SHOOTER_MAX_POWER);
            } else if (gamepad1.b) {
                shooterMotor.setPower(SHOOTER_MIN_POWER);
            } else {
                shooterMotor.setPower(0.0);
            }



            // -----------------------------
            // Hood Fixed
            // -----------------------------
            turretHood.setPosition(HOOD_FIXED_POS);

            // -----------------------------
            // Intake
            // -----------------------------
            double intakePower = gamepad1.right_trigger - gamepad1.left_trigger;
            frontIntake.setPower(intakePower);

            // -----------------------------
            // Telemetry
            // -----------------------------
            telemetry.addData("Target Visible", targetVisible);
            telemetry.addData("tx", "%.2f", tx);
            telemetry.addData("ty", "%.2f", ty);
            telemetry.addData("Turret Pos", "%.3f", turretServo.getPosition());
            telemetry.addData("Aligned?", aligned);
            telemetry.addData("Shooter Power", "%.3f", shooterMotor.getPower());
            telemetry.addData("Manual Override", manualOverrideActive);
            telemetry.update();

            dashTelemetry.addData("tx", tx);
            dashTelemetry.addData("ty", ty);
            dashTelemetry.addData("TurretPos", turretServo.getPosition());
            dashTelemetry.addData("Aligned?", aligned);
            dashTelemetry.update();
        }
    }
}
