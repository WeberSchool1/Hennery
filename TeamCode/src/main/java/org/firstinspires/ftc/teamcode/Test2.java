package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;


@TeleOp(name = "Test2", group = "TeleOp")
public class Test2 extends LinearOpMode {

    // -------------------------
    // Hardware
    // -------------------------
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor shooterMotor, frontIntake;
    private Servo turretServo, turretHood;
    private Limelight3A limelight;

    // -------------------------
    // Tunable constants (starting values — tune these for your robot)
    // -------------------------
    // Drive
    private static final double DRIVE_SCALE = 0.85;

    // Turret mapping
    private static final double SERVO_MIN_POS = 0.0;
    private static final double SERVO_MAX_POS = 1.0;
    private static final double SERVO_MIN_ANGLE_DEG = -60.0; // logical safe range (tune to your mech limits)
    private static final double SERVO_MAX_ANGLE_DEG = 60.0;
    private static final double TURRET_KP = 0.12;            // smoothing factor (0.05..0.3)
    private static final double TURRET_ALIGNMENT_SERVO_TOL = 0.01; // servo units (0..1) tolerance ~1%

    // Shooter power mapping (linear distance → power)
    private static final double SHOOTER_MIN_POWER = 0.35;   // safe close-shot baseline
    private static final double SHOOTER_MAX_POWER = 0.95;   // cap
    private static final double SHOOTER_DISTANCE_SCALE = 0.0025; // power per inch (start small)

    // Hood mapping (servo position = HOOD_MIN_POS + distance * HOOD_DISTANCE_SCALE)
    private static final double HOOD_MIN_POS = 0.12;   // safe low hood
    private static final double HOOD_MAX_POS = 0.65;   // max (don't go to 1.0 if mechanical limits)
    private static final double HOOD_DISTANCE_SCALE = 0.0020; // servo units per inch

    // Limelight mounting geometry (yours)
    private static final double CAMERA_HEIGHT = 13.3;    // inches (you gave)
    private static final double TARGET_HEIGHT = 36.0;    // inches (AprilTag top)
    private static final double CAMERA_ANGLE_DEG = 19.0; // mounting pitch (you gave)

    // Shooter encoder mapping: default guess. PLEASE measure your motor ticks/sec vs power and replace this value.
    // We'll initially use this to compute a target velocity (ticks/sec) from the computed power.
    private static final double DEFAULT_TICKS_PER_POWER = 6000.0; // placeholder — measure & replace
    private double ticksPerPower = DEFAULT_TICKS_PER_POWER;

    // Up-to-speed tolerance (fraction of target velocity)
    private static final double UP_TO_SPEED_TOLERANCE_FRACT = 0.08; // 8% tolerance

    // Manual turret jog step (servo units per loop when D-pad held)
    private static final double MANUAL_SERVO_STEP = 0.01;

    // -------------------------
    // Runtime fields
    // -------------------------
    private int lastShooterTicks = 0;
    private long lastTimeMs = 0;
    private double currentVelocityTicksPerSec = 0.0;
    private double targetVelocityTicksPerSec = 0.0;
    private boolean isUpToSpeed = false;

    // track manual override
    private boolean manualOverrideActive = false;

    @Override
    public void runOpMode() throws InterruptedException {

        FtcDashboard dashboard = FtcDashboard.getInstance();


        // Hardware map
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");

        shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");
        frontIntake  = hardwareMap.get(DcMotor.class, "frontIntake");

        turretServo = hardwareMap.get(Servo.class, "turretTurnLeft");
        turretHood  = hardwareMap.get(Servo.class, "turretHood");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Shooter motor setup: reversed (you asked)
        shooterMotor.setDirection(DcMotor.Direction.REVERSE);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize encoder tracking
        lastShooterTicks = shooterMotor.getCurrentPosition();
        lastTimeMs = System.currentTimeMillis();

        // Ensure Limelight pipeline is set to the pipeline index you use for AprilTags
        limelight.pipelineSwitch(5);

        telemetry.addLine("Test2 initialized. Press start.");
        telemetry.update();
        waitForStart();

        limelight.start();

        // main loop
        while (opModeIsActive()) {

            // -------------------------
            // Drive (simple mecanum linear)
            // left stick: translation, right stick X: rotation
            // -------------------------
            double driveY = -gamepad1.left_stick_y * DRIVE_SCALE; // forward
            double driveX =  gamepad1.left_stick_x * DRIVE_SCALE; // strafe
            double turn   =  gamepad1.right_stick_x * DRIVE_SCALE; // rotate

            double fl = driveY + driveX + turn;
            double fr = driveY - driveX - turn;
            double bl = driveY - driveX + turn;
            double br = driveY + driveX - turn;

            // normalize
            double max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)), Math.max(Math.abs(bl), Math.abs(br)));
            if (max > 1.0) {
                fl /= max; fr /= max; bl /= max; br /= max;
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

            // -------------------------
            // Distance calculation (inches)
            // distance = (targetHeight - cameraHeight) / tan(cameraAngle + ty)
            // -------------------------
            double distanceInches = 0.0;
            if (targetVisible) {
                double denom = Math.tan(Math.toRadians(CAMERA_ANGLE_DEG + ty));
                // protect against divide by zero
                if (Math.abs(denom) > 1e-6) {
                    distanceInches = (TARGET_HEIGHT - CAMERA_HEIGHT) / denom;
                } else {
                    distanceInches = 0.0;
                }
            }

            // -------------------------
            // Turret auto-align (only when X is held) + manual D-pad override
            // - compute desired servo position from tx
            // - if manual D-pad pressed -> manual override jogs turret
            // - otherwise when X held -> auto-align moves toward desired
            // -------------------------
            double desiredServoPos = turretServo.getPosition(); // default keep current
            if (targetVisible) {
                double logicalAngleDeg = -tx; // adjust sign if turret moves opposite
                logicalAngleDeg = Math.max(SERVO_MIN_ANGLE_DEG, Math.min(SERVO_MAX_ANGLE_DEG, logicalAngleDeg));
                double angRange = SERVO_MAX_ANGLE_DEG - SERVO_MIN_ANGLE_DEG;
                desiredServoPos = (logicalAngleDeg - SERVO_MIN_ANGLE_DEG) / angRange; // 0..1 mapping
                desiredServoPos = Math.max(SERVO_MIN_POS, Math.min(SERVO_MAX_POS, desiredServoPos));
            }

            // manual override via D-pad
            double currentServoPos = turretServo.getPosition();
            if (gamepad1.dpad_left) {
                manualOverrideActive = true;
                currentServoPos -= MANUAL_SERVO_STEP;
            } else if (gamepad1.dpad_right) {
                manualOverrideActive = true;
                currentServoPos += MANUAL_SERVO_STEP;
            } else {
                // if no dpad input, clear manual override flag (auto resumes)
                manualOverrideActive = false;
            }

            // If manual override active, set pos from D-pad; otherwise, if X is held, do auto align smoothing
            double nextServoPos;
            if (manualOverrideActive) {
                nextServoPos = currentServoPos;
            } else if (gamepad1.x && targetVisible) {
                // move toward desired gradually
                currentServoPos = turretServo.getPosition();
                double servoDelta = desiredServoPos - currentServoPos;
                nextServoPos = currentServoPos + servoDelta * TURRET_KP;
            } else {
                // neither manual nor X-held auto-align -> keep current position
                nextServoPos = turretServo.getPosition();
            }

            // clamp and set
            nextServoPos = Math.max(SERVO_MIN_POS, Math.min(SERVO_MAX_POS, nextServoPos));
            turretServo.setPosition(nextServoPos);

            // compute servo-based error and aligned bool
            double turretServoError = desiredServoPos - turretServo.getPosition();
            boolean aligned = targetVisible && Math.abs(turretServoError) <= TURRET_ALIGNMENT_SERVO_TOL;

            // -------------------------
            // Hood auto-adjust (only while shooting)
            // -------------------------
            boolean shootX = gamepad1.x; // distance-based auto (holds)
            boolean shootY = gamepad1.y; // max preset
            boolean shootB = gamepad1.b; // min preset
            boolean shooting = shootX || shootY || shootB;

            if (shooting) {
                // compute a hood position from distance (falls back to mid if no distance)
                double hoodDesired = (distanceInches > 0.0)
                        ? HOOD_MIN_POS + distanceInches * HOOD_DISTANCE_SCALE
                        : (HOOD_MIN_POS + HOOD_MAX_POS) / 2.0;
                hoodDesired = Math.max(HOOD_MIN_POS, Math.min(HOOD_MAX_POS, hoodDesired));
                turretHood.setPosition(hoodDesired);
            }

            // -------------------------
            // Shooter power from distance (linear mapping)
            // -------------------------
            double computedShooterPower = SHOOTER_MIN_POWER;
            if (targetVisible && distanceInches > 0.0) {
                computedShooterPower = SHOOTER_MIN_POWER + distanceInches * SHOOTER_DISTANCE_SCALE;
                if (computedShooterPower > SHOOTER_MAX_POWER) computedShooterPower = SHOOTER_MAX_POWER;
            }

            // -------------------------
            // Shooter encoder velocity (ticks/sec)
            // -------------------------
            int currentTicks = shooterMotor.getCurrentPosition();
            long nowMs = System.currentTimeMillis();
            long dt = nowMs - lastTimeMs;
            if (dt > 0) {
                currentVelocityTicksPerSec = ((double)(currentTicks - lastShooterTicks)) / (double)dt * 1000.0;
            }
            lastShooterTicks = currentTicks;
            lastTimeMs = nowMs;

            // target velocity mapping (power -> ticks/sec)
            targetVelocityTicksPerSec = computedShooterPower * ticksPerPower;

            // up-to-speed check
            double allowedDelta = Math.max(1.0, UP_TO_SPEED_TOLERANCE_FRACT * targetVelocityTicksPerSec);
            isUpToSpeed = Math.abs(currentVelocityTicksPerSec - targetVelocityTicksPerSec) <= allowedDelta;

            // -------------------------
            // Shooter control: X (distance), Y (max), B (min)
            // -------------------------
            if (shootX) {
                if (targetVisible && distanceInches > 0.0) shooterMotor.setPower(computedShooterPower);
                else shooterMotor.setPower(0.0);
            } else if (shootY) {
                shooterMotor.setPower(SHOOTER_MAX_POWER);
            } else if (shootB) {
                shooterMotor.setPower(SHOOTER_MIN_POWER);
            } else {
                shooterMotor.setPower(0.0);
            }

            // -------------------------
            // Intake proportional
            // -------------------------
            double intakePower = gamepad1.right_trigger - gamepad1.left_trigger;
            frontIntake.setPower(intakePower);

            // -------------------------
            // Telemetry (everything to tune)
            // -------------------------
            telemetry.addData("TargetVisible", targetVisible);
            telemetry.addData("tx", "%.2f", tx);
            telemetry.addData("ty", "%.2f", ty);
            telemetry.addData("Distance (in)", "%.1f", distanceInches);

            telemetry.addLine("--- Turret ---");
            telemetry.addData("DesiredServoPos", "%.3f", desiredServoPos);
            telemetry.addData("TurretPos", "%.3f", turretServo.getPosition());
            telemetry.addData("ServoError", "%.4f", turretServoError);
            telemetry.addData("Aligned(servo tol)", aligned);
            telemetry.addData("ManualOverride", manualOverrideActive);

            telemetry.addLine("--- Shooter/Hood ---");
            telemetry.addData("ComputedPower", "%.3f", computedShooterPower);
            telemetry.addData("ShooterPowerCmd", "%.3f", shooterMotor.getPower());
            telemetry.addData("TargetVel (ticks/s)", "%.1f", targetVelocityTicksPerSec);
            telemetry.addData("CurrentVel (ticks/s)", "%.1f", currentVelocityTicksPerSec);
            telemetry.addData("UpToSpeed?", isUpToSpeed);
            telemetry.addData("HoodPos", "%.3f", turretHood.getPosition());

            telemetry.addLine("--- Controls ---");
            telemetry.addData("X", "auto align+distance-shot");
            telemetry.addData("Y", "preset max shot");
            telemetry.addData("B", "preset min shot");
            telemetry.addData("DPad L/R", "manual turret jog");
            telemetry.addData("RT/LT", "intake in/out (proportional)");

            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("tx", tx);
            packet.put("ty", ty);
            packet.put("distanceInches", distanceInches);
            packet.put("servoPos", turretServo.getPosition());
            packet.put("hoodPos", turretHood.getPosition());
            packet.put("shooterVel", currentVelocityTicksPerSec);
            packet.put("targetVel", targetVelocityTicksPerSec);
            packet.put("upToSpeed", isUpToSpeed);
            dashboard.sendTelemetryPacket(packet);

        }
    }
}
