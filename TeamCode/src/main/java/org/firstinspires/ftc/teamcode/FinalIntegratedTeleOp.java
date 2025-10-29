package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "FinalIntegratedTeleOp", group = "TeleOp")
public class FinalIntegratedTeleOp extends LinearOpMode {

    // ----------------------
    // Hardware
    // ----------------------
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor shooterMotor;
    private DcMotor frontIntake;

    private Servo turretServo; // hardware name "turretTurnRight" - standard servo (0..1)
    private Servo turretHood;  // hardware name "turretHood"

    private IMU imu;
    private Limelight3A limelight;

    // ----------------------
    // Tunable constants (start values — tune these)
    // ----------------------
    // Shooter power mapping (linear)
    private static final double SHOOTER_MIN_POWER = 0.40;    // power for very close shots
    private static final double SHOOTER_MAX_POWER = 0.68;    // max power for far shots
    private static final double SHOOTER_DISTANCE_SCALE = 0.015; // how fast power increases with -ty

    // Convert shooter power -> "target velocity" scale (ticks/sec). This is a rough scaling factor.
    // Tune this to match how encoder ticks/sec correspond to power on your shooter flywheel.
    private static final double SHOOTER_POWER_TO_TICKS_PER_SEC = 6000.0; // starting guess for GoBILDA 6000RPM

    // Turret alignment smoothing (simple P-style move toward desired servo pos)
    // We compute desired servo position from tx and then move current position a fraction toward it each loop.
    private static final double SERVO_MIN_ANGLE_DEG = -45.0;   // logical min angle (deg) for mapping tx -> servo
    private static final double SERVO_MAX_ANGLE_DEG = 45.0;    // logical max angle (deg)
    private static final double SERVO_MIN_POS = 0.0;           // servo position corresponding to SERVO_MIN_ANGLE_DEG
    private static final double SERVO_MAX_POS = 1.0;           // servo position corresponding to SERVO_MAX_ANGLE_DEG
    private static final double TURRET_KP = 0.2; // smoothing factor (how aggressively servo moves to desired) — start small (0.02..0.15)

    // Hood mapping (linear); adjust scale to tune
    private static final double HOOD_MIN_POS = 0.09;   // servo position for close shot (tune)
    private static final double HOOD_MAX_POS = 0.5;   // servo position for far shot (tune)
    private static final double HOOD_DISTANCE_SCALE = 0.005; // how hood position changes with -ty (tune)

    // up-to-speed tolerance (fraction)
    private static final double UP_TO_SPEED_TOLERANCE = 1.2; // 12% tolerance

    // turret alignment tolerance (degrees)
    private static final double TURRET_ALIGNMENT_TOL = 0.9;

    // ----------------------
    // Runtime tracking variables
    // ----------------------
    private int lastShooterTicks = 0;
    private long lastTimeMs = 0;
    private double currentVelocityTicksPerSec = 0.0;
    private double targetVelocityTicksPerSec = 0.0;
    private boolean isUpToSpeed = false;

    @Override
    public void runOpMode() throws InterruptedException {
        // ---- Hardware map ----
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");

        shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");
        frontIntake  = hardwareMap.get(DcMotor.class, "frontIntake");

        turretServo = hardwareMap.get(Servo.class, "turretTurnRight");
        turretHood  = hardwareMap.get(Servo.class, "turretHood");

        imu = hardwareMap.get(IMU.class, "imu");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // ---- Motor setup ----
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        shooterMotor.setDirection(DcMotor.Direction.REVERSE);  // you told me shooter is reversed
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // so we can read encoder position

        frontIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ---- Initialize encoder tracking ----
        lastShooterTicks = shooterMotor.getCurrentPosition();
        lastTimeMs = System.currentTimeMillis();

        // ---- Limelight pipeline (ensure pipeline index is correct for your Tag detector) ----
        // Change pipeline index if your AprilTag pipeline is not 5
        limelight.pipelineSwitch(5);

        // Do NOT move servos multiple times here — we'll update them inside the loop after start.
        telemetry.addLine("Initialized. Press start when ready.");
        telemetry.update();
        waitForStart();

        limelight.start();

        // Main loop
        while (opModeIsActive()) {

            // -------------------------
            // Drive: field-centric mecanum
            // -------------------------
            double driveX = -gamepad1.left_stick_x;   // strafe
            double driveY = -gamepad1.left_stick_y;   // forward/back
            double turn   = -gamepad1.right_stick_x;  // rotation

            YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
            double headingRad = Math.toRadians(angles.getYaw());

            // rotate joystick vector by -heading (so controls are field-centric)
            double tmpX = driveX * Math.cos(headingRad) - driveY * Math.sin(headingRad);
            double tmpY = driveX * Math.sin(headingRad) + driveY * Math.cos(headingRad);
            driveX = tmpX;
            driveY = tmpY;

            double fl = driveY + driveX + turn;
            double fr = driveY - driveX - turn;
            double bl = driveY - driveX + turn;
            double br = driveY + driveX - turn;

            // normalize
            double max = Math.max(Math.abs(fl),
                          Math.max(Math.abs(fr),
                          Math.max(Math.abs(bl), Math.abs(br))));
            if (max > 1.0) {
                fl /= max; fr /= max; bl /= max; br /= max;
            }

            frontLeft.setPower(fl);
            frontRight.setPower(fr);
            backLeft.setPower(bl);
            backRight.setPower(br);

            // -------------------------
            // Limelight: get target info
            // -------------------------
            LLResult ll = limelight.getLatestResult();
            boolean targetVisible = (ll != null && ll.isValid());
            double tx = targetVisible ? ll.getTx() : 0.0; // horizontal angle (deg)
            double ty = targetVisible ? ll.getTy() : 0.0; // vertical angle (deg)
            double ta = (ll != null) ? ll.getTa() : 0.0;  // area (if you want)

            // Debug: if ll == null, show that
            if (ll == null) {
                telemetry.addLine("Limelight result == null");
            } else {
                telemetry.addData("LL valid", ll.isValid());
            }
// -------------------------
// Turret: auto-align
// -------------------------
            double desiredServoPos = turretServo.getPosition(); // default to current position
            if (targetVisible) {
                // Map Limelight horizontal offset (tx) to logical servo angle
                double logicalAngle = -tx; // flip if servo moves opposite direction
                logicalAngle = Math.max(SERVO_MIN_ANGLE_DEG, Math.min(SERVO_MAX_ANGLE_DEG, logicalAngle));

                // Map logical angle to servo position 0..1
                double angRange = SERVO_MAX_ANGLE_DEG - SERVO_MIN_ANGLE_DEG;
                desiredServoPos = (logicalAngle - SERVO_MIN_ANGLE_DEG) / angRange;
                desiredServoPos = Math.max(SERVO_MIN_POS, Math.min(SERVO_MAX_POS, desiredServoPos));
            }

// Smoothly move servo toward desired position
            double currentServoPos = turretServo.getPosition();
            double servoDelta = desiredServoPos - currentServoPos;
            double nextServoPos = currentServoPos + servoDelta * TURRET_KP;
            nextServoPos = Math.max(SERVO_MIN_POS, Math.min(SERVO_MAX_POS, nextServoPos));
            turretServo.setPosition(nextServoPos);

// Compute actual error (servo position units)
            double turretError = desiredServoPos - turretServo.getPosition();

// Only consider aligned if servo is physically near desired position
            boolean aligned = targetVisible && Math.abs(turretError) <= 0.01; // 0.01 = ~1% of servo range

            // -------------------------
            // Hood auto-adjust (linear mapping from -ty)
            // -------------------------
            boolean xPressed = gamepad1.x; // auto-distance shot
            boolean yPressed = gamepad1.y; // max distance
            boolean bPressed = gamepad1.b; // close distance

            boolean shooting = xPressed || yPressed || bPressed;

            if (shooting) {
                double hoodDesired = 0.5; // default
                if (targetVisible) {
                    // linear mapping from -ty
                    hoodDesired = 0.2 + (-ty + 15.0) * 0.02; // tune these values
                }
                // clamp to servo range
                hoodDesired = Math.max(HOOD_MIN_POS, Math.min(HOOD_MAX_POS, hoodDesired));
                turretHood.setPosition(hoodDesired);
            }



            // -------------------------
            // Shooter: compute target shooter power from Limelight distance (ty)
            // only set shooterMotor.setPower(...) while one of the shoot buttons (X / Y / B) is held
            // -------------------------
            double computedShooterPower = 0.0;
            if (targetVisible) {
                // linear mapping: base + scale * (-ty + offset)
                computedShooterPower = SHOOTER_MIN_POWER + (-ty + 10.0) * SHOOTER_DISTANCE_SCALE;
                computedShooterPower = Math.max(SHOOTER_MIN_POWER, Math.min(SHOOTER_MAX_POWER, computedShooterPower));
            }

            // Update encoder-based velocity measurement (works with any encoder-equipped motor)
            int currentTicks = shooterMotor.getCurrentPosition();
            long nowMs = System.currentTimeMillis();
            long dt = nowMs - lastTimeMs;
            if (dt > 0) {
                currentVelocityTicksPerSec = ((double)(currentTicks - lastShooterTicks)) / (double)dt * 1000.0;
            }
            lastShooterTicks = currentTicks;
            lastTimeMs = nowMs;

            // Determine desired target velocity in ticks/sec from shooter power
            // (this is a simple proportional mapping; tune SHOOTER_POWER_TO_TICKS_PER_SEC)
            targetVelocityTicksPerSec = computedShooterPower * SHOOTER_POWER_TO_TICKS_PER_SEC;

            // Decide if we're up to speed (within tolerance)
            isUpToSpeed = (Math.abs(currentVelocityTicksPerSec - targetVelocityTicksPerSec) <= (UP_TO_SPEED_TOLERANCE * Math.max(1.0, targetVelocityTicksPerSec)));

            // -------------------------
            // Shooting control:
            // - X (hold): distance-based auto shot (spin shooter to computed power; when upToSpeed shows true you may manually feed)
            // - Y (hold): preset MAX power (useful quick full-power)
            // - B (hold): preset MIN power (close shot)
            // The code DOES NOT automatically feed balls (per your request). It only spins the shooter and shows readiness.
            // -------------------------
            if (gamepad1.x) {
                // only do distance-based if target visible; otherwise do nothing
                if (targetVisible) {
                    shooterMotor.setPower(computedShooterPower);
                } else {
                    shooterMotor.setPower(0.0);
                }
            } else if (gamepad1.y) { // max preset
                shooterMotor.setPower(SHOOTER_MAX_POWER);
            } else if (gamepad1.b) { // min preset
                shooterMotor.setPower(SHOOTER_MIN_POWER);
            } else {
                // no shoot buttons -> stop shooter
                shooterMotor.setPower(0.0);
            }

            // -------------------------
            // Intake control (triggers proportional)
            // right_trigger - left_trigger
            // -------------------------
            double intakePower = gamepad1.right_trigger - gamepad1.left_trigger;
            // optional scale if you want to reduce top speed: intakePower *= 0.9;
            frontIntake.setPower(intakePower);

            // -------------------------
            // Telemetry (tuning values you'll want to watch)
            // -------------------------
            telemetry.addData("Target Visible", targetVisible);
            telemetry.addData("tx (deg)", "%.2f", tx);
            telemetry.addData("ty (deg)", "%.2f", ty);
            telemetry.addData("Aligned?", aligned);
            telemetry.addData("Desired Servo Pos", "%.3f", desiredServoPos);
            telemetry.addData("Current Servo Pos", "%.3f", turretServo.getPosition());
            telemetry.addData("Turret Error", "%.3f", turretError);
            telemetry.addData("Aligned?", aligned);

            telemetry.addLine("--- Shooter ---");
            telemetry.addData("Computed Power", "%.3f", computedShooterPower);
            telemetry.addData("Shooter Motor Power", "%.3f", shooterMotor.getPower());
            telemetry.addData("CurrentVel (ticks/s)", "%.1f", currentVelocityTicksPerSec);
            telemetry.addData("TargetVel (ticks/s)", "%.1f", targetVelocityTicksPerSec);
            telemetry.addData("UpToSpeed?", isUpToSpeed);

            telemetry.addLine("--- Controls ---");
            telemetry.addData("X", "distance-shot (hold)");
            telemetry.addData("Y", "preset max (hold)");
            telemetry.addData("B", "preset min (hold)");
            telemetry.addData("RT/LT", "intake in/out (proportional)");

            telemetry.update();
        }
    }
}
