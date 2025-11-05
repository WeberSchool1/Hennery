package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;

@Config
@TeleOp(name = "TurretAutoAlignPID", group = "Test")
public class TurretAutoAlignPID extends LinearOpMode {

    private Servo turretServo;       // 300° continuous servo
    private Limelight3A limelight;   
    private IMU imu;

    // PID constants (tune these)
    private static  double kP = 0.01;
    private static  double kI = 0.0001;
    private static  double kD = 0.002;

    private static  double SERVO_MIN_POS = 0.0;
    private static  double SERVO_MAX_POS = 1.0;

    private static  double HEADING_OFFSET = 0.0; // adjust if needed

    private double integral = 0;
    private double lastError = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        turretServo = hardwareMap.get(Servo.class, "turretTurnLeft");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        imu = hardwareMap.get(IMU.class, "imu");

        limelight.pipelineSwitch(5); // ensure AprilTag pipeline
        telemetry.addLine("Initialized. Press start.");
        telemetry.update();
        waitForStart();
        limelight.start();

        while (opModeIsActive()) {
            LLResult ll = limelight.getLatestResult();
            boolean targetVisible = (ll != null && ll.isValid());

            if (targetVisible) {
                double tx = ll.getTx(); // horizontal offset from crosshair
                double heading = imu.getRobotYawPitchRollAngles().getYaw(); // robot heading in degrees

                // Calculate desired angle to target relative to robot
                double targetAngle = heading + tx + HEADING_OFFSET;

                // Map targetAngle (-150° to +150°) to servo position (0.0–1.0)
                double logicalAngle = Math.max(-150, Math.min(150, targetAngle));
                double desiredPos = (logicalAngle + 150) / 300.0; // 0->-150°, 1->+150°

                // PID control
                double currentPos = turretServo.getPosition();
                double error = desiredPos - currentPos;

                integral += error;
                double derivative = error - lastError;
                double output = kP * error + kI * integral + kD * derivative;

                double nextPos = currentPos + output;

                // Clamp
                nextPos = Math.max(SERVO_MIN_POS, Math.min(SERVO_MAX_POS, nextPos));
                turretServo.setPosition(nextPos);

                lastError = error;

                telemetry.addData("Target Visible", targetVisible);
                telemetry.addData("TX (deg)", tx);
                telemetry.addData("Heading (deg)", heading);
                telemetry.addData("Target Angle", targetAngle);
                telemetry.addData("Desired Pos", desiredPos);
                telemetry.addData("Current Pos", currentPos);
                telemetry.addData("Next Pos", nextPos);
                telemetry.addData("Error", error);
            } else {
                telemetry.addData("Target Visible", false);
            }

            telemetry.update();
        }
    }
}
