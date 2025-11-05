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
 * PhysicsShooterPIDv2
 * - Computes distance from Limelight (using camera height and mount angle).
 * - Uses fixed hood servo position (HOOD_FIXED_POS) and physics projectile formula
 *   to compute the required launch speed.
 * - Converts required launch speed -> motor ticks/sec and uses I-dominant velocity PID
 *   plus feedforward to reach required speed.
 * - Press Y to "relocalize" (store last Ty). Hold A to spin shooter (safety).
 *
 * Tuneable via FTC Dashboard (@Config variables).
 */
@Config
@TeleOp(name = "PhysicsShooterPIDv2", group = "Test")
public class PhysicsShooterPIDv2 extends LinearOpMode {

    // ---------------- Hardware names ----------------
    public static String SHOOTER_MOTOR_NAME = "shooterMotor";
    public static String HOOD_SERVO_NAME = "turretHood";
    public static String LIMELIGHT_NAME = "limelight";

    // ---------------- Physical / geometry (editable) ----------------
    // Your inputs:
    // camera lens height (inches) and camera mounting angle (deg) you just gave
    public static double CAMERA_HEIGHT_IN = 12.54;
    public static double TARGET_HEIGHT_IN = 36.0;         // 3 ft AprilTag top
    public static double CAMERA_MOUNT_ANGLE_DEG = 70.0;   // 70 degrees from horizontal

    // flywheel radius in meters (you gave 96 mm -> 0.096 m)
    public static double FLYWHEEL_RADIUS_M = 0.096;
    // gear ratio motor -> wheel (1:1)
    public static double GEAR_RATIO = 1.0;
    // encoder ticks per motor revolution (set to your encoder)
    public static double ENCODER_TICKS_PER_REV = 537.7;

    // ---------------- Hood (fixed) ----------------
    // Use a fixed hood servo position (you asked for 0.8)
    public static double HOOD_FIXED_POS = 0.8;
    // mapping servo pos -> hood angle (deg). Adjust if your hood mapping differs.
    public static double HOOD_MIN_ANGLE_DEG = 0.0;
    public static double HOOD_MAX_ANGLE_DEG = 60.0;
    public static double HOOD_MIN_POS = 0.0;
    public static double HOOD_MAX_POS = 1.0;

    // ---------------- Velocity PID & feedforward (Dashboard-tuneable) ----------------
    public static double kP = 0.0;
    public static double kI = 0.0002;   // start here per article (I-only first)
    public static double kD = 0.0;

    // feedforward scaling
    public static double MAX_MOTOR_RPM = 6000.0;     // set to your motor free-run rpm if known
    public static double FEEDFORWARD_GAIN = 1.0;     // multiplier to increase baseline feedforward
    // scale to convert pid ticks/sec -> power. Tweak if mapping seems off.
    public static double TICKS_TO_POWER_SCALE = 1.0 / (ENCODER_TICKS_PER_REV * (MAX_MOTOR_RPM / 60.0));

    // safety limits
    public static double MAX_MOTOR_POWER = 1.0;
    public static double MIN_MOTOR_POWER = 0.0;

    // up-to-speed tolerance fraction
    public static double UP_TO_SPEED_TOL_FRAC = 0.05; // 5%

    // anti-windup clamp for integral
    public static double INTEGRAL_CLAMP = 1e7;

    // ---------------- runtime ----------------
    private DcMotor shooterMotor, frontIntake, turretSpin, backIntake;
    private Servo turretHood;
    private Limelight3A limelight;

    // PID state
    private double integral = 0.0;
    private double lastError = 0.0;

    // encoder tracking
    private int lastTicks = 0;
    private long lastTimeMs = 0;

    // stored ty (manual relocalize)
    private double storedTy = Double.NaN;

    @Override
    public void runOpMode() throws InterruptedException {
        shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");
        frontIntake = hardwareMap.get(DcMotor.class, "frontIntake");
        turretSpin =  hardwareMap.get(DcMotor.class, "turretOne");

        turretHood = hardwareMap.get(Servo.class, "turretHood");

        backIntake = hardwareMap.get(DcMotor.class, "backIntake");

        // Dashboard telemetry
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        // initialize
        turretHood.setPosition(HOOD_FIXED_POS); // fixed hood per your request

        lastTicks = shooterMotor.getCurrentPosition();
        lastTimeMs = System.currentTimeMillis();

        telemetry.addLine("PhysicsShooterPIDv2 ready.");
        telemetry.addLine("Hold A to spin shooter. Press Y to store current Limelight ty (relocalize).");
        telemetry.update();

        waitForStart();
        limelight.start();

        while (opModeIsActive()) {
            long nowMs = System.currentTimeMillis();
            double dt = (nowMs - lastTimeMs) / 1000.0;
            if (dt <= 0) dt = 1e-3;
            lastTimeMs = nowMs;

            // --- Limelight
            LLResult ll = limelight.getLatestResult();
            boolean targetVisible = (ll != null && ll.isValid());
            double ty = targetVisible ? ll.getTy() : Double.NaN;

            // manual re-localize/store Ty on Y
            if (gamepad1.y && targetVisible) {
                storedTy = ty;
            }

            // choose which Ty to use: current visible or stored
            double useTy = !Double.isNaN(storedTy) && !targetVisible ? storedTy : ty;

            // compute distance (meters) from camera geometry if we have a Ty value
            double distanceM = Double.NaN;
            if (!Double.isNaN(useTy)) {
                // convert inches to meters
                double camH = CAMERA_HEIGHT_IN * 0.0254;
                double tgtH = TARGET_HEIGHT_IN * 0.0254;
                double mountRad = Math.toRadians(CAMERA_MOUNT_ANGLE_DEG);
                double angleRad = mountRad + Math.toRadians(useTy);
                double tan = Math.tan(angleRad);
                if (Math.abs(tan) > 1e-6) {
                    double d = (tgtH - camH) / tan;
                    if (d > 0) distanceM = d;
                }
            }

            // --- Hood: fixed position (map position->angle)
            double hoodPos = HOOD_FIXED_POS;
            turretHood.setPosition(hoodPos);
            double hoodRatio = (hoodPos - HOOD_MIN_POS) / (HOOD_MAX_POS - HOOD_MIN_POS);
            hoodRatio = Math.max(0.0, Math.min(1.0, hoodRatio));
            double hoodAngleDeg = HOOD_MIN_ANGLE_DEG + hoodRatio * (HOOD_MAX_ANGLE_DEG - HOOD_MIN_ANGLE_DEG);
            double hoodAngleRad = Math.toRadians(hoodAngleDeg);

            // --- Physics: compute required launch velocity (m/s) for this distance and hood angle
            double requiredLaunchV = Double.NaN;
            if (!Double.isNaN(distanceM)) {
                // formula:
                // v = sqrt( g * d^2 / (2 * (h_target - h_cam - d * tan(theta)) * cos^2(theta)) )
                double camH = CAMERA_HEIGHT_IN * 0.0254;
                double tgtH = TARGET_HEIGHT_IN * 0.0254;
                double numerator = 9.81 * distanceM * distanceM;
                double denom = 2.0 * (tgtH - camH - distanceM * Math.tan(hoodAngleRad)) * Math.pow(Math.cos(hoodAngleRad), 2);
                if (denom > 1e-9) {
                    double v = Math.sqrt(numerator / denom);
                    if (!Double.isNaN(v) && Double.isFinite(v)) requiredLaunchV = v;
                }
            }

            // --- convert required launch velocity -> required encoder ticks/sec
            double requiredTicksPerSec = Double.NaN;
            if (!Double.isNaN(requiredLaunchV)) {
                // wheel rev/sec = v / (2*pi*r)
                double wheelRevPerSec = requiredLaunchV / (2.0 * Math.PI * FLYWHEEL_RADIUS_M);
                double motorRevPerSec = wheelRevPerSec * GEAR_RATIO;
                requiredTicksPerSec = motorRevPerSec * ENCODER_TICKS_PER_REV;
            }

            // --- measure current ticks/sec
            int currentTicks = shooterMotor.getCurrentPosition();
            double measuredTicksPerSec = (currentTicks - lastTicks) / dt;
            lastTicks = currentTicks;

            // --- PID (I-dominant) over ticks/sec
            double pidTicksOutput = 0.0;
            double feedForwardPower = 0.0;
            double finalPower = 0.0;
            boolean haveTarget = !Double.isNaN(requiredTicksPerSec);

            if (haveTarget) {
                double errorTicks = requiredTicksPerSec - measuredTicksPerSec;
                integral += errorTicks * dt;

                // anti-windup
                if (integral > INTEGRAL_CLAMP) integral = INTEGRAL_CLAMP;
                if (integral < -INTEGRAL_CLAMP) integral = -INTEGRAL_CLAMP;

                double derivative = (errorTicks - lastError) / dt;
                pidTicksOutput = kP * errorTicks + kI * integral + kD * derivative;
                lastError = errorTicks;

                // Feedforward: estimate baseline motor power
                double maxMotorRevPerSec = MAX_MOTOR_RPM / 60.0;
                // required wheel rev/sec (recompute safely)
                double requiredWheelRevPerSec = !Double.isNaN(requiredLaunchV) ? (requiredLaunchV / (2.0 * Math.PI * FLYWHEEL_RADIUS_M)) : Double.NaN;
                if (!Double.isNaN(requiredWheelRevPerSec)) {
                    double requiredMotorRevPerSec = requiredWheelRevPerSec * GEAR_RATIO;
                    feedForwardPower = FEEDFORWARD_GAIN * (requiredMotorRevPerSec / maxMotorRevPerSec);
                    feedForwardPower = Math.max(0.0, Math.min(1.0, feedForwardPower));
                }

                // Convert PID ticks/sec to power via scale (tunable)
                double pidPower = pidTicksOutput * TICKS_TO_POWER_SCALE;

                finalPower = feedForwardPower + pidPower;
                if (finalPower > MAX_MOTOR_POWER) finalPower = MAX_MOTOR_POWER;
                if (finalPower < MIN_MOTOR_POWER) finalPower = MIN_MOTOR_POWER;
            }

            // Only run shooter while A is held (safety)
            if (gamepad1.a && haveTarget) {
                shooterMotor.setPower(finalPower);
            } else {
                shooterMotor.setPower(0.0);
                // optionally freeze integral if not running:
                // integral = 0; // leave enabled if you prefer integral to accumulate even while not running
            }

            // up-to-speed check
            boolean upToSpeed = false;
            if (haveTarget) {
                upToSpeed = Math.abs(requiredTicksPerSec - measuredTicksPerSec) <= (UP_TO_SPEED_TOL_FRAC * Math.max(1.0, requiredTicksPerSec));
            }

            // Telemetry
            telemetry.addData("targetVisible", targetVisible);
            telemetry.addData("useTy (deg)", !Double.isNaN(useTy) ? String.format("%.2f", useTy) : "NaN");
            telemetry.addData("distance (m)", !Double.isNaN(distanceM) ? String.format("%.2f", distanceM) : "NaN");
            telemetry.addData("hoodAngle (deg)", String.format("%.2f", hoodAngleDeg));
            telemetry.addData("required launch v (m/s)", !Double.isNaN(requiredLaunchV) ? String.format("%.2f", requiredLaunchV) : "NaN");
            telemetry.addData("required ticks/sec", !Double.isNaN(requiredTicksPerSec) ? String.format("%.1f", requiredTicksPerSec) : "NaN");
            telemetry.addData("measured ticks/sec", String.format("%.1f", measuredTicksPerSec));
            telemetry.addData("feedforward", String.format("%.3f", feedForwardPower));
            telemetry.addData("pidTicksOut", String.format("%.3f", pidTicksOutput));
            telemetry.addData("finalPower", String.format("%.3f", finalPower));
            telemetry.addData("upToSpeed?", upToSpeed);
            telemetry.addData("storedTy", Double.isNaN(storedTy) ? "NaN" : String.format("%.2f", storedTy));
            telemetry.update();
        }
    }
}
