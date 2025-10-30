package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

@Config
@TeleOp(name = "Test7_TurretCR_PID_HoldHelper", group = "Test")
public class Test7_TurretCR_PID_HoldHelper extends LinearOpMode {

    // ----------------- Tunable PID Constants -----------------
    public static double kP = 0.015;
    public static double kI = 0.0001;
    public static double kD = 0.002;

    public static double MAX_SERVO_POWER = 0.5;       // Max power applied to CR servo
    public static double MANUAL_SERVO_POWER = 0.2;    // Power when using D-pad
    public static double HEADING_OFFSET = 0.0;        // If your turret is mounted rotated

    public static double ALIGNMENT_TOL_DEG = 1.5;     // Alignment tolerance in degrees
    public static double UPDATE_INTERVAL = 0.02;      // Loop integration interval (seconds)

    public static double DEGREES_PER_SEC_FULL_POWER = 300; // Adjust after measuring helper mode

    // ----------------- Hardware -----------------
    private Servo turretServo;
    private Limelight3A limelight;
    private IMU imu;

    // ----------------- PID Tracking -----------------
    private double integral = 0;
    private double lastError = 0;

    // Approximate turret position in degrees (-150 to +150)
    private double turretLogicalPos = 0;

    @Override
    public void runOpMode() {

        turretServo = hardwareMap.get(Servo.class, "turretTurnLeft"); // Continuous
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        imu = hardwareMap.get(IMU.class, "imu");

        limelight.pipelineSwitch(5); // AprilTag pipeline
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        telemetry.addLine("Turret CR PID Hold Helper Initialized. Press Start");
        telemetry.update();
        waitForStart();
        limelight.start();

        boolean manualOverride = false;

        long lastTime = System.nanoTime();

        while (opModeIsActive()) {

            // ----------------- Timing -----------------
            long nowTime = System.nanoTime();
            double dt = (nowTime - lastTime) / 1e9; // seconds
            if (dt <= 0) dt = UPDATE_INTERVAL;
            lastTime = nowTime;

            // ----------------- Limelight -----------------
            LLResult ll = limelight.getLatestResult();
            boolean targetVisible = (ll != null && ll.isValid());
            double tx = targetVisible ? ll.getTx() : 0.0;

            // ----------------- Reset turret logical position -----------------
            if (gamepad1.y) {
                turretLogicalPos = 0;
                integral = 0;
                lastError = 0;
            }

            // ----------------- Helper mode to measure degrees/sec -----------------
            if (gamepad1.a) {
                turretServo.setPosition(0.5); // fixed test power
                // In Dashboard, watch "Logical Pos" changing to estimate degrees/sec
                turretLogicalPos += 0.5 * DEGREES_PER_SEC_FULL_POWER * dt;
            } else {
                // ----------------- Manual override -----------------
                double servoPower = 0;
                if (gamepad1.dpad_left) {
                    manualOverride = true;
                    servoPower = -MANUAL_SERVO_POWER;
                } else if (gamepad1.dpad_right) {
                    manualOverride = true;
                    servoPower = MANUAL_SERVO_POWER;
                } else {
                    manualOverride = false;
                }

                // ----------------- Auto-align PID -----------------
                boolean aligned = false;
                if (gamepad1.x && targetVisible && !manualOverride) {
                    double heading = imu.getRobotYawPitchRollAngles().getYaw();
                    double targetAngle = heading + tx + HEADING_OFFSET;

                    // Clamp logical angle
                    double logicalAngle = Math.max(-150, Math.min(150, targetAngle));

                    // PID
                    double error = logicalAngle - turretLogicalPos;
                    integral += error * dt;
                    double derivative = (error - lastError) / dt;
                    servoPower = kP * error + kI * integral + kD * derivative;

                    // Clamp power
                    servoPower = Math.max(-MAX_SERVO_POWER, Math.min(MAX_SERVO_POWER, servoPower));
                    lastError = error;

                    // Check if aligned
                    aligned = Math.abs(error) <= ALIGNMENT_TOL_DEG;

                    if (aligned) {
                        servoPower = 0; // hold position
                        integral = 0;
                    }
                }

                // ----------------- Apply power -----------------
                turretServo.setPosition(servoPower);

                // ----------------- Update estimated position -----------------
                turretLogicalPos += servoPower * DEGREES_PER_SEC_FULL_POWER * dt; 
                turretLogicalPos = Math.max(-150, Math.min(150, turretLogicalPos));
            }

            // ----------------- Telemetry -----------------
            telemetry.addData("Target Visible", targetVisible);
            telemetry.addData("TX", tx);
            telemetry.addData("Turret Logical Pos", "%.2f", turretLogicalPos);
            telemetry.addData("Servo Power", "%.3f", turretServo.getPosition());
            telemetry.addData("Aligned?", gamepad1.x && targetVisible && !manualOverride && Math.abs(lastError) <= ALIGNMENT_TOL_DEG);
            telemetry.addData("Manual Override", manualOverride);
            telemetry.addData("Integral", integral);
            telemetry.addData("Error", lastError);
            telemetry.update();
        }
    }
}
