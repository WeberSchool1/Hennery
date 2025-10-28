package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "FieldCentricDrive", group = "Drive")
public class FieldCentricDrive extends LinearOpMode {

    // Motors
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // IMU
    private IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        // Hardware mapping
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");

        imu = hardwareMap.get(IMU.class, "imu");

        // Set motor directions (adjust if needed for your robot)
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        // Optional: brake mode for better control
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for start
        telemetry.addLine("Ready for Field-Centric Drive");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // Read joystick inputs
            double driveX = -gamepad1.left_stick_x; // Strafing
            double driveY = -gamepad1.left_stick_y; // Forward/back
            double turn   = -gamepad1.right_stick_x; // Rotation

            // Read current heading from IMU
            YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
            double heading = Math.toRadians(angles.getYaw()); // Convert to radians

            // Rotate joystick inputs to be field-centric
            double tempX = driveX * Math.cos(heading) - driveY * Math.sin(heading);
            double tempY = driveX * Math.sin(heading) + driveY * Math.cos(heading);

            driveX = tempX;
            driveY = tempY;

            // Calculate motor powers for mecanum drive
            double frontLeftPower  = driveY + driveX + turn;
            double frontRightPower = driveY - driveX - turn;
            double backLeftPower   = driveY - driveX + turn;
            double backRightPower  = driveY + driveX - turn;

            // Normalize powers if any exceed 1.0
            double max = Math.max(Math.abs(frontLeftPower), Math.max(Math.abs(frontRightPower),
                       Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))));
            if (max > 1.0) {
                frontLeftPower  /= max;
                frontRightPower /= max;
                backLeftPower   /= max;
                backRightPower  /= max;
            }

            // Set motor powers
            frontLeft.setPower(frontLeftPower);
            frontRight.setPower(frontRightPower);
            backLeft.setPower(backLeftPower);
            backRight.setPower(backRightPower);

            // Telemetry for debugging
            telemetry.addData("Heading (deg)", Math.toDegrees(heading));
            telemetry.addData("FL", frontLeftPower);
            telemetry.addData("FR", frontRightPower);
            telemetry.addData("BL", backLeftPower);
            telemetry.addData("BR", backRightPower);
            telemetry.update();
        }
    }
}
