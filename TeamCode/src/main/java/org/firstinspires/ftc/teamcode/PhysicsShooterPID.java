package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Physics-based shooter velocity controller (based on the "DECODE Shooter" article).
 *
 * Features:
 * - Compute distance from Limelight ty, limelight height & mounting angle.
 * - Search hood angle (within allowed hood angles) for the minimal required launch velocity.
 * - Convert required launch velocity -> motor ticks/sec using wheel radius and gearing.
 * - Velocity PID (I-dominant) to reach & hold required ticks/sec. PID terms are Dashboard-tunable.
 * - Press A to spin shooter (hold to run). Press B to "relocalize" the last measured ty (store it).
 *
 * Notes / tuning:
 * - Set ENCODER_TICKS_PER_REV to your motor encoder ticks per revolution.
 * - Set DEGREES_PER_SECOND_FULL_POWER if you later use CR servos; here we use positional hood servo.
 * - Use FTC Dashboard to tune kI first (I-only), then add a small kP if desired.
 */
@Config
@TeleOp(name = "PhysicsShooterPID", group = "Test")
public class PhysicsShooterPID extends LinearOpMode {

    // --------------- Hardware names (change if your config differs) ---------------
    public static String SHOOTER_MOTOR_NAME = "shooterMotor";
    public static String HOOD_SERVO_NAME = "turretHood";
    public static String LIMELIGHT_NAME = "limelight"; // usually "limelight"

    // ----------------------- Physical / mech parameters (user-supplied) -----------------------
    // Flywheel radius (meters). You said 96 mm radius -> 0.096 m
    public static double FLYWHEEL_RADIUS_M = 0.096; // 96 mm -> 0.096 m

    // Gear ratio motor -> flywheel (motor_rev * GEAR_RATIO = flywheel_rev). For 1:1 use 1.0
    public static double GEAR_RATIO = 1.0;

    // Encoder ticks per motor revolution (set to your encoder)
    public static double ENCODER_TICKS_PER_REV = 28; // common GoBILDA-ish default; change if needed

    // Limelight + field geometry (inches -> will converted to meters in code)
    public static double LIMELIGHT_HEIGHT_IN = 12.54;
    public static double TARGET_HEIGHT_IN = 36.0; // AprilTag top height (3 ft) you gave
    public static double LIMELIGHT_MOUNT_ANGLE_DEG = 70.0;

    // ----------------------- Hood servo angle mapping -----------------------
    // These define the physical hood angle range (degrees) that correspond to servo positions 0..1.
    // E.g. if your hood servo at pos 0.0 = -10 deg and pos 1.0 = 60 deg, set these accordingly.
    public static double HOOD_MIN_ANGLE_DEG = 0.0;    // degrees (lower/close shot)
    public static double HOOD_MAX_ANGLE_DEG = 60.0;   // degrees (higher/far shot)
    public static double HOOD_MIN_POS = .8;          // servo pos at HOOD_MIN_ANGLE_DEG
    public static double HOOD_MAX_POS = .45;          // servo pos at HOOD_MAX_ANGLE_DEG

    // ----------------------- Search / physics parameters -----------------------
    // Search resolution when looking for the hood angle that minimizes required velocity
    public static double HOOD_SEARCH_STEP_DEG = 0.5; // degrees step for search (smaller = slower but more accurate)

    // Gravity (m/s^2)
    private static final double G = 9.81;

    // ----------------------- Shooter velocity PID (I-dominant recommended) -----------------------
    // Start with kP = 0, kI > 0, kD = 0 per article; you can tune in Dashboard.
    public static double kP = 0.0;
    public static double kI = 0.0002;
    public static double kD = 0.0;

    // Safety / limits
    public static double MAX_MOTOR_POWER = 1.0;
    public static double MIN_MOTOR_POWER = 0.0;

    // Tolerance for "up to speed" (fraction of target velocity)
    public static double UP_TO_SPEED_TOL_FRAC = 0.05; // 5%

    // ----------------------- runtime state -----------------------
    private DcMotor shooterMotor;
    private Servo hoodServo;
    private Limelight3A limelight;

    // PID state
    private double integral = 0.0;
    private double lastError = 0.0;

    // encoder tracking (we compute ticks/sec manually)
    private int lastTicks = 0;
    private long lastTimeMs = 0;

    // "manual relocalize" stored ty value (deg). Use B to store last seen ty.
    private double storedTy = Double.NaN;

    @Override
    public void runOpMode() throws InterruptedException {
        // Hardware map
        shooterMotor = hardwareMap.get(DcMotor.class, SHOOTER_MOTOR_NAME);
        shooterMotor.setDirection(DcMotor.Direction.REVERSE);
        hoodServo = hardwareMap.get(Servo.class, HOOD_SERVO_NAME);
        limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);

        // Shooter motor config
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Start dashboard telemetry
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        // convert geometry to SI (meters)
        final double limelightHeightM = LIMELIGHT_HEIGHT_IN * 0.0254;
        final double targetHeightM = TARGET_HEIGHT_IN * 0.0254;
        final double limelightMountAngleRad = Math.toRadians(LIMELIGHT_MOUNT_ANGLE_DEG);

        // initialize encoder tracking
        lastTicks = shooterMotor.getCurrentPosition();
        lastTimeMs = System.currentTimeMillis();

        telemetry.addLine("PhysicsShooterPID initialized. Press START.");
        telemetry.addLine("Hold A to spin shooter. Press B to store last ty (relocalize).");
        telemetry.update();
        waitForStart();
        limelight.start();

        while (opModeIsActive()) {
            long nowMs = System.currentTimeMillis();
            double dt = (nowMs - lastTimeMs) / 1000.0;
            if (dt <= 0) dt = 1e-3;
            lastTimeMs = nowMs;

            // Limelight reading
            LLResult ll = limelight.getLatestResult();
            boolean targetVisible = (ll != null && ll.isValid());
            double ty = targetVisible ? ll.getTy() : Double.NaN;

            // Manual re-localize (store ty) like article suggests
            if (gamepad1.b && targetVisible) {
                storedTy = ty;
            }

            // Use storedTy if available and target not currently visible
            double useTy = (!Double.isNaN(storedTy) && !targetVisible) ? storedTy : ty;

            // Compute horizontal distance (meters) from Limelight vertical angle ty
            double distanceM = Double.NaN;
            if (!Double.isNaN(useTy)) {
                // angle from horizontal = mount angle + ty
                double angleToTargetRad = limelightMountAngleRad + Math.toRadians(useTy);
                // horizontal distance d = (h_target - h_cam) / tan(angle)
                double denom = Math.tan(angleToTargetRad);
                if (Math.abs(denom) > 1e-6) {
                    distanceM = (targetHeightM - limelightHeightM) / denom;
                    if (distanceM < 0) distanceM = Double.NaN; // invalid geometry
                } else {
                    distanceM = Double.NaN;
                }
            }

            // Find hood angle (deg) which yields minimal required launch speed (search across hood angle range)
            double bestHoodDeg = Double.NaN;
            double bestLaunchV = Double.NaN;
            if (!Double.isNaN(distanceM)) {
                // search hood angle range [HOOD_MIN_ANGLE_DEG .. HOOD_MAX_ANGLE_DEG]
                for (double a = HOOD_MIN_ANGLE_DEG; a <= HOOD_MAX_ANGLE_DEG; a += HOOD_SEARCH_STEP_DEG) {
                    double theta = Math.toRadians(a);
                    // compute required launch velocity using formula:
                    // v = sqrt( g * d^2 / (2 * (h_target - h_cam - d * tan(theta)) * cos^2(theta)) )
                    double denom = 2.0 * (targetHeightM - limelightHeightM - distanceM * Math.tan(theta)) * Math.pow(Math.cos(theta), 2);
                    if (denom <= 0) {
                        continue; // invalid for this theta (would require imaginary velocity)
                    }
                    double numer = G * distanceM * distanceM;
                    double v = Math.sqrt(numer / denom);
                    if (Double.isNaN(v) || Double.isInfinite(v)) continue;
                    if (Double.isNaN(bestLaunchV) || v < bestLaunchV) {
                        bestLaunchV = v;
                        bestHoodDeg = a;
                    }
                }
            }

            // If no valid hood angle found (e.g. distance out of range), fall back to a safe hood angle midpoint
            if (Double.isNaN(bestHoodDeg)) {
                bestHoodDeg = (HOOD_MIN_ANGLE_DEG + HOOD_MAX_ANGLE_DEG) / 2.0;
            }
            // Map hood angle (deg) -> servo position (0..1)
            double hoodRatio = (bestHoodDeg - HOOD_MIN_ANGLE_DEG) / (HOOD_MAX_ANGLE_DEG - HOOD_MIN_ANGLE_DEG);
            double hoodPos = HOOD_MIN_POS + hoodRatio * (HOOD_MAX_POS - HOOD_MIN_POS);
            hoodPos = Math.max(Math.min(hoodPos, HOOD_MAX_POS), HOOD_MIN_POS);

            // Compute required wheel surface velocity (m/s) from bestLaunchV
            // The flywheel should impart a linear speed equal to v (approx). For a wheel of radius r:
            // wheel_rev_per_sec = v / (2*pi*r)
            double requiredWheelRevPerSec = Double.NaN;
            double requiredTicksPerSec = Double.NaN;
            double requiredMotorPowerEstimate = 0.0;
            if (!Double.isNaN(bestLaunchV)) {
                requiredWheelRevPerSec = bestLaunchV / (2.0 * Math.PI * FLYWHEEL_RADIUS_M);
                // convert to motor rev/sec using gear ratio: motorRev/sec = wheelRev/sec * gearRatio
                double motorRevPerSec = requiredWheelRevPerSec * GEAR_RATIO;
                requiredTicksPerSec = motorRevPerSec * ENCODER_TICKS_PER_REV;
            }

            // Encoder-based measured velocity (ticks/sec)
            int currentTicks = shooterMotor.getCurrentPosition();
            double measuredTicksPerSec = (currentTicks - lastTicks) / dt;
            lastTicks = currentTicks;

            // PID over ticks/sec: target = requiredTicksPerSec
            double pidOutput = 0.0;
            boolean haveTarget = !Double.isNaN(requiredTicksPerSec);
            if (haveTarget) {
                double error = requiredTicksPerSec - measuredTicksPerSec;
                integral += error * dt;
                // optional anti-windup: clamp integral to reasonable range
                double integralLimit = 100000.0; // large clamp, tuneable if desired
                if (integral > integralLimit) integral = integralLimit;
                if (integral < -integralLimit) integral = -integralLimit;

                double derivative = (error - lastError) / dt;
                pidOutput = kP * error + kI * integral + kD * derivative;
                lastError = error;

                // pidOutput is in "ticks/sec -> power" units. We need to map it to motor power 0..1.
                // Rather than guessing an unknown mapping, we convert requiredTicksPerSec to a nominal
                // "feed-forward" baseline and then add PID as a small correction. A simple and robust
                // approach: compute feedforward power proportional to required wheel speed:
                //   feedforward = requiredMotorRevPerSec / maxPossibleMotorRevPerSec
                // We'll declare a tunable MAX_MOTOR_RPM (or rev/sec) so feedforward scales nicely.

                // For now we compute a simple feedforward using a tunable MAX_MOTOR_RPM (rev/min).
            }

            // ---- feedforward mapping config (tweak in Dashboard) ----
            // Max motor rev/min at full power (approx; set conservatively)
            double maxMotorRPM = 6000.0; // default guess; change to match your motor free-run rpm if known
            double maxMotorRevPerSec = maxMotorRPM / 60.0;

            double feedForwardPower = 0.0;
            if (haveTarget && !Double.isNaN(requiredWheelRevPerSec)) {
                double requiredMotorRevPerSec = requiredWheelRevPerSec * GEAR_RATIO;
                feedForwardPower = requiredMotorRevPerSec / maxMotorRevPerSec;
                // clamp
                if (feedForwardPower > 1.0) feedForwardPower = 1.0;
                if (feedForwardPower < 0.0) feedForwardPower = 0.0;
            }

            // Combine feedforward + PID correction (scale PID to power by dividing by an empirical scale)
            // We need a mapping scale: ticks/sec -> power. We'll use ticksToPowerScale as a tunable config.
            double ticksToPowerScale = 1.0 / (ENCODER_TICKS_PER_REV * maxMotorRevPerSec); // crude default
            double pidPower = pidOutput * ticksToPowerScale;

            double finalPower = feedForwardPower + pidPower;

            // Clamp final power
            if (finalPower > MAX_MOTOR_POWER) finalPower = MAX_MOTOR_POWER;
            if (finalPower < MIN_MOTOR_POWER) finalPower = MIN_MOTOR_POWER;

            // Only run shooter while user holds A (safety)
            if (gamepad1.a && haveTarget) {
                shooterMotor.setPower(finalPower);
            } else {
                shooterMotor.setPower(0.0);
            }

            // Move hood servo to computed position
            hoodServo.setPosition(hoodPos);

            // Up-to-speed check (fraction)
            boolean upToSpeed = false;
            if (haveTarget) {
                upToSpeed = Math.abs(requiredTicksPerSec - measuredTicksPerSec) <= (UP_TO_SPEED_TOL_FRAC * Math.max(1.0, requiredTicksPerSec));
            }

            // Telemetry (dashboard + driver station)
            telemetry.addData("targetVisible", targetVisible);
            telemetry.addData("useTy (deg)", !Double.isNaN(useTy) ? String.format("%.2f", useTy) : "NaN");
            telemetry.addData("distance (m)", !Double.isNaN(distanceM) ? String.format("%.2f", distanceM) : "NaN");
            telemetry.addData("selectedHoodDeg", String.format("%.2f", bestHoodDeg));
            telemetry.addData("hoodPos", String.format("%.3f", hoodPos));
            telemetry.addData("required launch v (m/s)", !Double.isNaN(bestLaunchV) ? String.format("%.2f", bestLaunchV) : "NaN");
            telemetry.addData("required ticks/sec", !Double.isNaN(requiredTicksPerSec) ? String.format("%.1f", requiredTicksPerSec) : "NaN");
            telemetry.addData("measured ticks/sec", String.format("%.1f", measuredTicksPerSec));
            telemetry.addData("feedforward", String.format("%.3f", feedForwardPower));
            telemetry.addData("pidPower", String.format("%.4f", pidPower));
            telemetry.addData("finalPower", String.format("%.3f", finalPower));
            telemetry.addData("upToSpeed?", upToSpeed);
            telemetry.update();
        }
    }
}
