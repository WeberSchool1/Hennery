package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "MecanumTurretShooter", group = "TeleOp")
public class MecanumTurretShooter extends LinearOpMode {

    // Drive motors
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // Shooter and turret
    private DcMotor shooterMotor;
    private Servo turretServo; // hardware name: turretTurnLeft

    // Limelight
    private Limelight3A limelight;

    // IMU
    private IMU imu;

    // Turret limits
    private static final double SERVO_MIN_ANGLE = -45;
    private static final double SERVO_MAX_ANGLE = 45;
    private static final double ALIGNMENT_TOLERANCE = 2.0;

    private boolean shooterSpinning = false;
    private long shooterStartTime = 0;
    private final long SPINUP_DELAY_MS = 300; // time to reach full speed (adjust)


    @Override
    public void runOpMode() throws InterruptedException {

        // ===== Hardware mapping =====
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");

        shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");
        turretServo  = hardwareMap.get(Servo.class, "turretTurnLeft");

        imu = hardwareMap.get(IMU.class, "imu");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // ===== Motor setup =====
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        shooterMotor.setDirection(DcMotor.Direction.REVERSE); // reversed
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ===== Limelight setup =====
        limelight.pipelineSwitch(5);

        telemetry.addLine("Initialized. Waiting for start...");
        telemetry.update();
        waitForStart();

        limelight.start();

        // ===== Main loop =====
        while (opModeIsActive()) {

            // -------- Field-centric drive --------
            double driveX = -gamepad1.left_stick_x;
            double driveY = -gamepad1.left_stick_y;
            double turn   = -gamepad1.right_stick_x;

            YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
            double heading = Math.toRadians(angles.getYaw());

            double tempX = driveX * Math.cos(heading) - driveY * Math.sin(heading);
            double tempY = driveX * Math.sin(heading) + driveY * Math.cos(heading);

            driveX = tempX;
            driveY = tempY;

            double fl = driveY + driveX + turn;
            double fr = driveY - driveX - turn;
            double bl = driveY - driveX + turn;
            double br = driveY + driveX - turn;

            double max = Math.max(Math.abs(fl), Math.max(Math.abs(fr),
                    Math.max(Math.abs(bl), Math.abs(br))));
            if (max > 1.0) {
                fl /= max;
                fr /= max;
                bl /= max;
                br /= max;
            }

            frontLeft.setPower(fl);
            frontRight.setPower(fr);
            backLeft.setPower(bl);
            backRight.setPower(br);

            // -------- Limelight turret alignment --------
            LLResult result = limelight.getLatestResult();
            boolean targetVisible = (result != null && result.isValid());
            double tx = targetVisible ? result.getTx() : 0;
            double ty = targetVisible ? result.getTy() : 0;

            boolean aligned = false;
            if (targetVisible) {
                double clampedAngle = Math.max(SERVO_MIN_ANGLE, Math.min(SERVO_MAX_ANGLE, -tx));
                double servoPos = (clampedAngle - SERVO_MIN_ANGLE) / (SERVO_MAX_ANGLE - SERVO_MIN_ANGLE);
                turretServo.setPosition(servoPos);

                aligned = Math.abs(tx) <= ALIGNMENT_TOLERANCE;
            }

            // -------- Shooter power based on inverted ty --------
            if (targetVisible && aligned && gamepad1.a) {
                // Calculate shooter power
                double shooterPower = Math.min(0.8, 0.4 + (-ty + 10) * 0.0258);

                // Start spinning if not already
                if (!shooterSpinning) {
                    shooterMotor.setPower(shooterPower);
                    shooterSpinning = true;
                    shooterStartTime = System.currentTimeMillis();
                }

                // Check if spin-up time has passed
                if (System.currentTimeMillis() - shooterStartTime >= SPINUP_DELAY_MS) {
                    // Shooter is up to speed → fire ball
                    // Here you can activate a pusher servo or other mechanism
                    // Example: pusherServo.setPosition(1.0);
                }

            } else {
                // Stop shooter if button released
                shooterMotor.setPower(0);
                shooterSpinning = false;
            }

            // -------- Telemetry --------
            telemetry.addData("FL", fl);
            telemetry.addData("FR", fr);
            telemetry.addData("BL", bl);
            telemetry.addData("BR", br);
            telemetry.addData("Turret Servo", turretServo.getPosition());
            telemetry.addData("Target Visible", targetVisible);
            telemetry.addData("TX", tx);
            telemetry.addData("TY", ty);
            telemetry.addData("Aligned", aligned);
            telemetry.update();
        }
    }
}
