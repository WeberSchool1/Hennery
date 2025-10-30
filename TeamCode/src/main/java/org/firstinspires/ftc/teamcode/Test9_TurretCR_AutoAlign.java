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
@TeleOp(name = "Test9_TurretCR_AutoAlign", group = "Test")
public class Test9_TurretCR_AutoAlign extends LinearOpMode {

    // ----------------- Dashboard Editable Variables -----------------
    public static double kP = 0.015;
    public static double kI = 0.0001;
    public static double kD = 0.002;

    public static double MAX_SERVO_POWER = 0.5;       // Max CR servo power
    public static double ALIGNMENT_TOL_DEG = 1.5;     // stop within this tolerance
    public static double HEADING_OFFSET = 0.0;        // adjust if mounted rotated
    public static double DEGREES_PER_SEC_FULL_POWER = 300; // degrees/sec at full power

    // ----------------- Hardware -----------------
    private Servo turretServo;
    private Limelight3A limelight;
    private IMU imu;

    // ----------------- PID tracking -----------------
    private double integral = 0;
    private double lastError = 0;
    private double turretLogicalPos = 0; // estimated angle in degrees (-150 to +150)

    private boolean autoAlignActive = false;

    @Override
    public void runOpMode() {

        turretServo = hardwareMap.get(Servo.class, "turretTurnRight"); // continuous servo
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        imu = hardwareMap.get(IMU.class, "imu");

        limelight.pipelineSwitch(5); // AprilTag pipeline

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        telemetry.addLine("Turret CR AutoAlign Initialized. Press Start");
        telemetry.update();
        waitForStart();
        limelight.start();

        long lastTime = System.nanoTime();

        while (opModeIsActive()) {
            long nowTime = System.nanoTime();
            double dt = (nowTime - lastTime) / 1e9;
            if (dt <= 0) dt = 0.02;
            lastTime = nowTime;

            LLResult ll = limelight.getLatestResult();
            boolean targetVisible = (ll != null && ll.isValid());
            double tx = targetVisible ? ll.getTx() : 0.0;

            // ----------------- Start auto-align when A is pressed -----------------
            if (gamepad1.a) {
                autoAlignActive = true;
            }

            // ----------------- Auto-align PID -----------------
            double servoPower = 0;
            boolean aligned = false;

            if (autoAlignActive && targetVisible) {
                double heading = imu.getRobotYawPitchRollAngles().getYaw();
                double targetAngle = heading + tx + HEADING_OFFSET;

                double logicalAngle = Math.max(-150, Math.min(150, targetAngle));
                double error = logicalAngle - turretLogicalPos;

                // Stop if within tolerance
                if (Math.abs(error) <= ALIGNMENT_TOL_DEG) {
                    servoPower = 0;
                    autoAlignActive = false;
                    aligned = true;
                    integral = 0;
                } else {
                    // PID
                    integral += error * dt;
                    double derivative = (error - lastError) / dt;
                    servoPower = kP * error + kI * integral + kD * derivative;
                    servoPower = Math.max(-MAX_SERVO_POWER, Math.min(MAX_SERVO_POWER, servoPower));
                    lastError = error;
                }
            }

            // ----------------- Set continuous servo power -----------------
            turretServo.setPosition(servoPower);
            turretLogicalPos += servoPower * DEGREES_PER_SEC_FULL_POWER * dt;
            turretLogicalPos = Math.max(-150, Math.min(150, turretLogicalPos));

            // ----------------- Telemetry -----------------
            telemetry.addData("Target Visible", targetVisible);
            telemetry.addData("TX", tx);
            telemetry.addData("Turret Logical Pos", "%.2f", turretLogicalPos);
            telemetry.addData("Servo Power", "%.3f", servoPower);
            telemetry.addData("Auto-Align Active", autoAlignActive);
            telemetry.addData("Aligned?", aligned);
            telemetry.addData("PID Error", lastError);
            telemetry.addData("Integral", integral);
            telemetry.addData("Dashboard kP", kP);
            telemetry.addData("Dashboard kI", kI);
            telemetry.addData("Dashboard kD", kD);
            telemetry.addData("MAX_SERVO_POWER", MAX_SERVO_POWER);
            telemetry.addData("ALIGNMENT_TOL_DEG", ALIGNMENT_TOL_DEG);
            telemetry.update();
        }
    }
}
