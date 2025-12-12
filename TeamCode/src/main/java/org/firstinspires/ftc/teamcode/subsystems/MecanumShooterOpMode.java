package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.HoodRPMAdjuster;

@TeleOp(name = "Mecanum Shooter OpMode", group = "TeleOp")
public class MecanumShooterOpMode extends OpMode {

    // Drivetrain motors
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // Shooter motors and servo
    private DcMotorEx flyWheelLeft, flyWheelRight;
    private Servo hoodServo;

    // Limelight
    private Limelight3A limelight;

    // Shooter adjuster
    private HoodRPMAdjuster shooter;

    // Timing
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime shooterTimer = new ElapsedTime();

    // Shooter state
    private boolean shooterActive = false;
    private boolean lastAimButton = false;

    @Override
    public void init() {
        // Initialize drivetrain
        frontLeft = hardwareMap.get(DcMotor.class, "front left");
        frontRight = hardwareMap.get(DcMotor.class, "front right");
        backLeft = hardwareMap.get(DcMotor.class, "back left");
        backRight = hardwareMap.get(DcMotor.class, "back right");

        // Set motor directions (adjust based on your robot)
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set zero power behavior
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize shooter components
        flyWheelLeft = hardwareMap.get(DcMotorEx.class, "outtake Low");
        flyWheelRight = hardwareMap.get(DcMotorEx.class, "outtake High");
        flyWheelRight.setDirection(DcMotorSimple.Direction.REVERSE);
        hoodServo = hardwareMap.get(Servo.class, "hood");

        // Initialize Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); // Switch to pipeline 0
        limelight.start(); // Start polling

        // Initialize shooter system
        shooter = new HoodRPMAdjuster(flyWheelLeft, flyWheelRight, hoodServo);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Controls", "Left Stick: Drive | Right Stick: Strafe/Turn");
        telemetry.addData("Shooter", "Right Trigger: Auto-aim | A: Manual shoot");
        telemetry.update();
    }

    @Override
    public void loop() {
        // ==================== DRIVETRAIN ====================
        double drive = -gamepad1.left_stick_y;  // Forward/backward
        double strafe = gamepad1.left_stick_x;  // Left/right
        double turn = gamepad1.right_stick_x;   // Rotation

        // Apply speed multiplier (hold left bumper for slow mode)
        double speedMultiplier = gamepad1.left_bumper ? 0.4 : 1.0;

        // Calculate motor powers
        double frontLeftPower = (drive + strafe + turn) * speedMultiplier;
        double frontRightPower = (drive - strafe - turn) * speedMultiplier;
        double backLeftPower = (drive - strafe + turn) * speedMultiplier;
        double backRightPower = (drive + strafe - turn) * speedMultiplier;

        // Normalize powers if any exceed 1.0
        double maxPower = Math.max(Math.abs(frontLeftPower),
                Math.max(Math.abs(frontRightPower),
                        Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))));

        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;
        }

        // Set motor powers
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);

        // ==================== SHOOTER SYSTEM ====================

        // Auto-aim with Limelight (Right Trigger)
        boolean aimButton = gamepad1.right_trigger > 0.5;

        if (aimButton && !lastAimButton) {
            // Button just pressed - attempt auto-aim
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                boolean success = shooter.applyLimelightShot(result);

                if (success) {
                    shooterActive = true;
                    shooterTimer.reset();
                    telemetry.addData("Auto-Aim", "Locked on target!");
                } else {
                    telemetry.addData("Auto-Aim", "Failed - out of range");
                }
            } else {
                telemetry.addData("Auto-Aim", "No target detected");
            }
        }

        lastAimButton = aimButton;

        // Manual shoot button (A button) - uses default close-range settings
        if (gamepad1.a) {
            HoodRPMAdjuster.ShooterSettings closeShot = shooter.getSettingsForDistance(900.0);
            if (closeShot != null) {
                shooterActive = true;
                shooterTimer.reset();
            }
        }

        // Stop shooter (B button)
        if (gamepad1.b) {
            shooterActive = false;
            flyWheelLeft.setPower(0);
            flyWheelRight.setPower(0);
        }

        // ==================== TELEMETRY ====================

        telemetry.addData("Status", "Running: " + runtime.toString());
        telemetry.addData("", ""); // Blank line

        // Drivetrain info
        telemetry.addData("Drive Mode", gamepad1.left_bumper ? "SLOW" : "NORMAL");
        telemetry.addData("Drive", "%.2f", drive);
        telemetry.addData("Strafe", "%.2f", strafe);
        telemetry.addData("Turn", "%.2f", turn);
        telemetry.addData("", ""); // Blank line

        // Limelight info
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            double distance = shooter.getDistanceFromLimelight(result);
            HoodRPMAdjuster.ShooterSettings settings = shooter.getSettingsFromLimelight(result);

            telemetry.addData("Limelight", "TARGET LOCKED");
            telemetry.addData("Distance", "%.0f mm", distance);

            if (settings != null) {
                telemetry.addData("Target RPM", "%.0f", settings.rpm);
                telemetry.addData("Hood Angle", "%.1f°", settings.hoodAngleDegrees);
            }
        } else {
            telemetry.addData("Limelight", "No Target");
        }

        telemetry.addData("", ""); // Blank line

        // Shooter status
        telemetry.addData("Shooter Active", shooterActive);
        telemetry.addData("Current RPM", "%.0f", shooter.getCurrentRPM());
        telemetry.addData("At Speed", shooter.isAtSpeed() ? "YES ✓" : "NO");

        if (shooterActive && shooter.isAtSpeed()) {
            telemetry.addData("READY TO FIRE", "✓✓✓");
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        // Stop all motors
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        flyWheelLeft.setPower(0);
        flyWheelRight.setPower(0);

        // Stop Limelight
        limelight.stop();
    }
}