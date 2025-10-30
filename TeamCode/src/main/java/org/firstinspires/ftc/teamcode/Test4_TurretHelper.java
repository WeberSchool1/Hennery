package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;

import com.acmerobotics.dashboard.config.Config;

@Config
@TeleOp(name = "Test4_TurretHelper", group = "Test")
public class Test4_TurretHelper extends LinearOpMode {

    public static double kP = 0.01;
    public static double kI = 0.0001;
    public static double kD = 0.002;

    public static double SERVO_MIN_POS = 0.0;
    public static double SERVO_MAX_POS = 1.0;

    public static double HEADING_OFFSET = 0.0;
    public static double MANUAL_SERVO_STEP = 0.01;

    public static double MAX_LOGICAL_ANGLE = 150;
    public static double MIN_LOGICAL_ANGLE = -150;

    private Servo turretServo;
    private Limelight3A limelight;
    private IMU imu;

    private double integral = 0;
    private double lastError = 0;

    @Override
    public void runOpMode() {

        turretServo = hardwareMap.get(Servo.class, "turretTurnLeft");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        imu = hardwareMap.get(IMU.class, "imu");

        limelight.pipelineSwitch(5); // ensure AprilTag pipeline

        telemetry.addLine("Turret Helper Initialized. Press start.");
        telemetry.update();
        waitForStart();
        limelight.start();

        boolean manualOverrideActive = false;

        while (opModeIsActive()) {

            LLResult ll = limelight.getLatestResult();
            boolean targetVisible = (ll != null && ll.isValid());
            double tx = targetVisible ? ll.getTx() : 0.0;

            double desiredPos = turretServo.getPosition(); // default keep current

            if (targetVisible) {
                double heading = imu.getRobotYawPitchRollAngles().getYaw();
                double targetAngle = heading + tx + HEADING_OFFSET;
                targetAngle = Math.max(MIN_LOGICAL_ANGLE, Math.min(MAX_LOGICAL_ANGLE, targetAngle));
                double angRange = MAX_LOGICAL_ANGLE - MIN_LOGICAL_ANGLE;
                desiredPos = (targetAngle - MIN_LOGICAL_ANGLE) / angRange;
                desiredPos = Math.max(SERVO_MIN_POS, Math.min(SERVO_MAX_POS, desiredPos));
            }

            // Manual override via D-pad
            double currentPos = turretServo.getPosition();
            if (gamepad1.dpad_left) {
                manualOverrideActive = true;
                currentPos -= MANUAL_SERVO_STEP;
            } else if (gamepad1.dpad_right) {
                manualOverrideActive = true;
                currentPos += MANUAL_SERVO_STEP;
            } else {
                manualOverrideActive = false;
            }

            double nextPos;
            double pidOutput = 0;

            if (manualOverrideActive) {
                nextPos = currentPos;
            } else if (targetVisible) {
                // PID computation for testing
                double error = desiredPos - currentPos;
                integral += error;
                double derivative = error - lastError;
                pidOutput = kP * error + kI * integral + kD * derivative;
                nextPos = currentPos + pidOutput;
                nextPos = Math.max(SERVO_MIN_POS, Math.min(SERVO_MAX_POS, nextPos));
                lastError = error;
            } else {
                nextPos = turretServo.getPosition();
            }

            turretServo.setPosition(nextPos);

            // Telemetry for tuning
            double turretError = desiredPos - turretServo.getPosition();
            telemetry.addData("Target Visible", targetVisible);
            telemetry.addData("TX (deg)", tx);
            telemetry.addData("Desired Pos", desiredPos);
            telemetry.addData("Current Pos", turretServo.getPosition());
            telemetry.addData("Next Pos", nextPos);
            telemetry.addData("PID Output", pidOutput);
            telemetry.addData("Error", turretError);
            telemetry.addData("Manual Override?", manualOverrideActive);
            telemetry.update();
        }
    }
}
