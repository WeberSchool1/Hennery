package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "ShooterHoodAutoPID_Button", group = "Test")
public class ShooterHoodAutoPID_Button extends LinearOpMode {

    // ----- Hardware -----
    public DcMotor shooterMotor;
    public Servo turretHood;
    public Limelight3A limelight;

    // ----- PID (start with I-only per article) -----
    public static double kP = 0.0;
    public static double kI = 0.0001;
    public static double kD = 0.0;

    // ----- Shooter target scaling -----
    public static double SHOOTER_POWER_TO_TICKS_PER_SEC = 6000; // adjust per motor
    public static double UP_TO_SPEED_TOLERANCE = 0.05; // 5% tolerance

    // ----- Hood mapping -----
    public static double HOOD_MIN_POS = 0.12;
    public static double HOOD_MAX_POS = 0.5;
    public static double HOOD_DISTANCE_SCALE = 0.005; // linear mapping from -ty

    // ----- Runtime variables -----
    private double integral = 0;
    private double lastError = 0;
    private int lastTicks = 0;
    private long lastTime = 0;
    private boolean isUpToSpeed = false;

    // ----- Limelight tracking -----
    private double lastTy = 0; // store last measured ty

    @Override
    public void runOpMode() throws InterruptedException {

        shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turretHood = hardwareMap.get(Servo.class, "turretHood");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(5);

        telemetry.addLine("Initialized. Press start.");
        telemetry.update();
        waitForStart();
        limelight.start();

        lastTicks = shooterMotor.getCurrentPosition();
        lastTime = System.currentTimeMillis();

        while (opModeIsActive()) {

            LLResult ll = limelight.getLatestResult();
            boolean targetVisible = (ll != null && ll.isValid());

            // ----- Manual re-localization -----
            if (gamepad1.b && targetVisible) {
                lastTy = ll.getTy(); // update stored distance
            }

            // ----- Compute desired shooter power from distance -----
            double desiredPower = 0.0;
            if (targetVisible || lastTy != 0) {
                double ty = (targetVisible) ? ll.getTy() : lastTy;
                desiredPower = 0.4 + (-ty + 10) * HOOD_DISTANCE_SCALE; // base 0.4
                desiredPower = Math.max(0.4, Math.min(0.72, desiredPower));
            }

            // ----- Encoder-based velocity -----
            int currentTicks = shooterMotor.getCurrentPosition();
            long now = System.currentTimeMillis();
            double dt = (now - lastTime) / 1000.0;
            double velocity = (currentTicks - lastTicks) / dt;

            lastTicks = currentTicks;
            lastTime = now;

            // ----- PID -----
            double targetVelocity = desiredPower * SHOOTER_POWER_TO_TICKS_PER_SEC;
            double error = targetVelocity - velocity;

            integral += error * dt;
            double derivative = (error - lastError) / dt;
            double output = kP * error + kI * integral + kD * derivative;
            lastError = error;

            // ----- Shooter control: only spin if A is held -----
            if (gamepad1.a) {
                shooterMotor.setPower(output);
            } else {
                shooterMotor.setPower(0.0);
            }

            // ----- Check up-to-speed -----
            isUpToSpeed = Math.abs(error) <= UP_TO_SPEED_TOLERANCE * Math.max(1.0, targetVelocity);

            // ----- Hood position -----
            double hoodPos = 0.5; // default
            if (targetVisible || lastTy != 0) {
                double ty = (targetVisible) ? ll.getTy() : lastTy;
                hoodPos = 0.2 + (-ty + 15) * HOOD_DISTANCE_SCALE;
            }
            hoodPos = Math.max(HOOD_MIN_POS, Math.min(HOOD_MAX_POS, hoodPos));
            turretHood.setPosition(hoodPos);

            // ----- Telemetry -----
            telemetry.addData("Target Visible", targetVisible);
            telemetry.addData("Ty (deg)", "%.2f", (targetVisible ? ll.getTy() : lastTy));
            telemetry.addData("Desired Power", "%.3f", desiredPower);
            telemetry.addData("Shooter Velocity", "%.1f", velocity);
            telemetry.addData("Target Velocity", "%.1f", targetVelocity);
            telemetry.addData("PID Output", "%.3f", output);
            telemetry.addData("UpToSpeed?", isUpToSpeed);
            telemetry.addData("Hood Pos", "%.3f", hoodPos);
            telemetry.addData("Shooter Running?", gamepad1.a);
            telemetry.addData("Relocalize (B pressed)", lastTy);
            telemetry.update();
        }
    }
}
