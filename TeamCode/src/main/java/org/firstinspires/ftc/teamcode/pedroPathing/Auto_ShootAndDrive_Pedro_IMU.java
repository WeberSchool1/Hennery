package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

// Pedro Pathing imports
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name = "Auto_ShootAndDrive_Pedro_IMU", group = "Autonomous")
public class Auto_ShootAndDrive_Pedro_IMU extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor shooterMotor, turretSpin, backIntake;
    private Limelight3A limelight;
    private IMU imu;

    // Pedro follower
    private Follower follower;

    // Turret + shooter constants
    private final double kP_TURRET = 0.02;
    private final double ALIGN_TOLERANCE = 1.0; // degrees
    private final double CAMERA_HEIGHT = 12.54; // inches
    private final double TARGET_HEIGHT = 36.0;  // top of AprilTag
    private final double CAMERA_ANGLE = 70.0;   // degrees up
    private final double MIN_DISTANCE = 20.0;
    private final double MAX_DISTANCE = 100.0;
    private final double MIN_POWER = 0.45;
    private final double MAX_POWER = 0.75;
    private final long SHOOTER_SPINUP_MS = 800;
    private final long FEED_DURATION_MS = 700;

    @Override
    public void runOpMode() throws InterruptedException {
        // --- Hardware map ---
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");
        turretSpin = hardwareMap.get(DcMotor.class, "turretOne");
        backIntake = hardwareMap.get(DcMotor.class, "backIntake");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        imu = hardwareMap.get(IMU.class, "imu");

        // --- Set motor directions ---
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        shooterMotor.setDirection(DcMotor.Direction.REVERSE);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // --- Initialize IMU ---
        imu = hardwareMap.get(IMU.class, "imu");
        imu.resetYaw();


        // --- Limelight setup ---
        limelight.pipelineSwitch(5);

        // --- Pedro follower setup ---
        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));

        telemetry.addLine("Auto_ShootAndDrive_Pedro_IMU Ready");
        telemetry.update();

        waitForStart();
        limelight.start();

        // --- Check for AprilTag ---
        LLResult ll = limelight.getLatestResult();
        boolean tagSeen = ll != null && ll.isValid();

        if (tagSeen) {
            telemetry.addLine("AprilTag found, aligning turret...");
            telemetry.update();

            double tx = ll.getTx();
            double ty = ll.getTy();

            // Align turret
            while (opModeIsActive()) {
                ll = limelight.getLatestResult();
                if (ll == null || !ll.isValid()) break;
                tx = ll.getTx();
                if (Math.abs(tx) < ALIGN_TOLERANCE) {
                    turretSpin.setPower(0);
                    break;
                }
                double turretPower = kP_TURRET * tx;
                turretPower = clamp(turretPower, -0.4, 0.4);
                turretSpin.setPower(turretPower);
                telemetry.addData("Aligning", "tx=%.2f", tx);
                telemetry.update();
            }
            turretSpin.setPower(0);

            // Shooter power based on distance
            double distance = (TARGET_HEIGHT - CAMERA_HEIGHT) /
                    Math.tan(Math.toRadians(CAMERA_ANGLE + ty));
            distance = Math.max(MIN_DISTANCE, Math.min(MAX_DISTANCE, distance));
            double shooterPower = MIN_POWER + (distance - MIN_DISTANCE) /
                    (MAX_DISTANCE - MIN_DISTANCE) * (MAX_POWER - MIN_POWER);

            telemetry.addData("Distance (in)", distance);
            telemetry.addData("Shooter Power", shooterPower);
            telemetry.update();

            shooterMotor.setPower(shooterPower);
            sleep(SHOOTER_SPINUP_MS);

            // Feed projectile
            backIntake.setPower(0.8);
            sleep(FEED_DURATION_MS);
            backIntake.setPower(0.0);
            shooterMotor.setPower(0.0);
        } else {
            telemetry.addLine("No AprilTag visible, skipping shooting.");
            telemetry.update();
        }

        // --- Drive forward 30 inches ---
        final double WHEEL_DIAMETER_INCHES = 4.0; // adjust for your wheels
        final int TICKS_PER_REV = 537; // adjust for your motors
        final double GEAR_RATIO = 1; // motor to wheel ratio
        final double TICKS_PER_INCH = (TICKS_PER_REV * GEAR_RATIO) / (Math.PI * WHEEL_DIAMETER_INCHES);

        int targetTicks = (int)(30 * TICKS_PER_INCH);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setTargetPosition(targetTicks);
        frontRight.setTargetPosition(targetTicks);
        backLeft.setTargetPosition(targetTicks);
        backRight.setTargetPosition(targetTicks);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double drivePower = 0.5;
        frontLeft.setPower(drivePower);
        frontRight.setPower(drivePower);
        backLeft.setPower(drivePower);
        backRight.setPower(drivePower);

        while(opModeIsActive() &&
                (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy())) {
            // Optional: read IMU yaw for telemetry
            double yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            telemetry.addData("Driving forward", "Yaw=%.2f", yaw);
            telemetry.update();
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        telemetry.addLine("Autonomous complete!");
        telemetry.update();

    }

    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}
