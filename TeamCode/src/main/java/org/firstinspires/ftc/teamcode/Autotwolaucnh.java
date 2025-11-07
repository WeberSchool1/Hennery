package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;

@Autonomous(name="AutoTwolaunch", group="Autonomous")
public class Autotwolaucnh extends LinearOpMode {
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

    private final double deadbandDeg = 0.5;


    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor shooterMotor, turretSpin, backIntake, frontIntake;
    private Limelight3A limelight;
    private IMU imu;
    private Follower follower;


    @Override
    public void runOpMode() throws InterruptedException {

        shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");
        turretSpin = hardwareMap.get(DcMotor.class, "turretOne");
        backIntake = hardwareMap.get(DcMotor.class, "backIntake");
        frontIntake = hardwareMap.get(DcMotor.class, "frontIntake");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        imu = hardwareMap.get(IMU.class, "imu");

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        // --- setup ---
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addLine("Ready to run");
        telemetry.update();

        waitForStart();

        // --- 1️⃣ Spin up shooter motor ---
        double targetPower = 0.67; // target power
        shooterMotor.setPower(targetPower);

        double targetVelocity = 6000 * targetPower; // rough guess: 6000 ticks/sec max
        double tolerance = 0.10; // 10% tolerance
        int lastTicks = shooterMotor.getCurrentPosition();
        long lastTime = System.currentTimeMillis();
        double velocity = 0;

        // --- 2️⃣ Wait until shooter is up to speed ---
        while (opModeIsActive()) {
            int currentTicks = shooterMotor.getCurrentPosition();
            long now = System.currentTimeMillis();
            double dt = (now - lastTime) / 1000.0;
            if (dt > 0) velocity = (currentTicks - lastTicks) / dt;

            lastTicks = currentTicks;
            lastTime = now;

            boolean upToSpeed = Math.abs(velocity - targetVelocity) <= (targetVelocity * tolerance);

            telemetry.addData("Velocity (ticks/sec)", velocity);
            telemetry.addData("Target Velocity", targetVelocity);
            telemetry.addData("Up to speed?", upToSpeed);
            telemetry.update();

            if (upToSpeed) break; // exit loop when shooter is ready
        }

        // --- 3️⃣ Feed ball with backIntake ---
        backIntake.setPower(1.0);
        sleep(1500); // feed for 1.5 seconds
        backIntake.setPower(0.0);


        frontIntake.setPower(1.0);
        sleep(1000);

        backIntake.setPower(1.0);
        sleep(1000);
        backIntake.setPower(0.0);


        // --- 4️⃣ Optionally drive forward ---
        double drivePower = 0.3;
        frontLeft.setPower(drivePower);
        frontRight.setPower(drivePower);
        backLeft.setPower(drivePower);
        backRight.setPower(drivePower);
        sleep(2000);

        // --- 5️⃣ Stop everything ---
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        shooterMotor.setPower(0);
        backIntake.setPower(0);
        frontIntake.setPower(0);

        telemetry.addLine("Done!");
        telemetry.update();
    }
}


