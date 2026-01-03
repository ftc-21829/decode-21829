package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.rowanmcalpin.nextftc.core.command.CommandManager;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup;

@TeleOp(name = "TeleopTest-newTracking")
public class TeleopTest extends OpMode {
    AllMechCopy robot;
    boolean Outtake;
    Gamepad currentGamepad1, previousGamepad1, currentGamepad2, previousGamepad2;
    private final FtcDashboard dash = FtcDashboard.getInstance();

    @Override
    public void init() {
        robot = new AllMechCopy(hardwareMap, gamepad1, gamepad2);

        currentGamepad1 = new Gamepad();
        currentGamepad2 = new Gamepad();
        previousGamepad1 = new Gamepad();
        previousGamepad2 = new Gamepad();

        telemetry.addData("Status", "Initialized");
        telemetry.addLine("=== CONTROLS ===");
        telemetry.addData("GP2 Square", "Toggle Turret Tracking");
        telemetry.addData("GP2 Cross", "Turn Off Turret");
        telemetry.addData("GP2 Triangle", "Vision Tracking Mode");
        telemetry.addData("GP2 Circle", "Field-Relative Mode");
        telemetry.addData("GP1 R1", "Toggle Turret (alt)");
        telemetry.update();
    }

    @Override
    public void start() {
        robot.follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        // Update gamepad states
        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);
        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);

        // Update follower (Pinpoint odometry)
        robot.follower.update();

        // ========== DRIVETRAIN CONTROL ==========
        double y = -gamepad2.left_stick_y;
        double x = -gamepad2.left_stick_x * 1.1;
        double rx = -gamepad2.right_stick_x;
        robot.follower.setTeleOpDrive(y, x, rx, true);

        // ========== UPDATE LIMELIGHT ==========
        robot.updateLimelightData();

        // ========== TRACKING MODE SWITCHING ==========
        // GP2 Triangle: Switch to vision-based tracking
        if (currentGamepad2.triangle && !previousGamepad2.triangle) {
            robot.USE_FIELD_RELATIVE_TRACKING = false;
        }

        // GP2 Circle: Switch to field-relative tracking
        if (currentGamepad2.circle && !previousGamepad2.circle) {
            robot.USE_FIELD_RELATIVE_TRACKING = true;
        }

        // ========== TURRET TRACKING CONTROLS ==========
        if (currentGamepad2.square && !previousGamepad2.square) {
            robot.setTurretTrackingActive(!robot.isTurretTrackingActive());
        }

        if (currentGamepad2.cross && !previousGamepad2.cross) {
            robot.setTurretTrackingActive(false);
        }

        // Alternative turret toggle on GP1 R1
        if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
            CommandManager.INSTANCE.scheduleCommand(robot.turretOn());
        }

        // ========== UPDATE TURRET ==========
        robot.updateTurretTracking();

        // ========== INTAKE CONTROLS ==========
        if (gamepad1.dpad_right) {
            CommandManager.INSTANCE.scheduleCommand(robot.intakeOn());
        }
        if (gamepad1.dpad_left) {
            CommandManager.INSTANCE.scheduleCommand(robot.intakeOff());
        }
        if (gamepad1.dpad_up) {
            CommandManager.INSTANCE.scheduleCommand(
                    new ParallelGroup(
                            robot.transferOn(),
                            robot.intakeOn(),
                            robot.transferCheck(),
                            robot.doorOpen()
                    )
            );
        }
        if (gamepad1.dpad_down) {
            CommandManager.INSTANCE.scheduleCommand(robot.IntakeOut());
        }

        // ========== TRANSFER CONTROLS ==========
        if (gamepad1.left_bumper) {
            CommandManager.INSTANCE.scheduleCommand(robot.transferOff());
        }

        // ========== OUTTAKE/SHOOTER CONTROLS ==========
        if (gamepad1.triangle) {
            Outtake = true;
        }
        if (gamepad1.circle) {
            Outtake = false;
        }

        if (Outtake) {
            robot.periodicShooterUpdateAndApplyPID();
        } else {
            CommandManager.INSTANCE.scheduleCommand(robot.OuttakeOff());
        }

        // ========== BUTT KICKER ==========
        if (gamepad1.right_stick_button) {
            CommandManager.INSTANCE.scheduleCommand(robot.ButtKicker());
        }

        // Run command manager
        CommandManager.INSTANCE.run();

        // ========== TELEMETRY ==========
        telemetry.addLine("=== TRACKING MODE ===");
        telemetry.addData("Mode", robot.USE_FIELD_RELATIVE_TRACKING ?
                "FIELD-RELATIVE (Pinpoint)" : "VISION-BASED (Limelight)");

        telemetry.addLine();
        telemetry.addLine("=== ROBOT POSE (Pinpoint) ===");
        Pose pose = robot.follower.getPose();
        telemetry.addData("X", "%.2f in", pose.getX());
        telemetry.addData("Y", "%.2f in", pose.getY());
        telemetry.addData("Heading", "%.1f°", Math.toDegrees(pose.getHeading()));

        telemetry.addLine();
        telemetry.addLine("=== TARGET INFO ===");
        telemetry.addData("Target", "(%.1f, %.1f)", robot.TARGET_X, robot.TARGET_Y);
        telemetry.addData("Distance", "%.1f in", robot.getDistanceToTarget());

        telemetry.addLine();
        telemetry.addLine("=== TURRET STATUS ===");
        telemetry.addData("Tracking", robot.isTurretTrackingActive() ? "ACTIVE" : "OFF");
        telemetry.addData("Aligned", robot.isTurretAligned() ? "YES ✓" : "NO");

        if (robot.USE_FIELD_RELATIVE_TRACKING) {
            telemetry.addData("Status", "Tracking field position");
        } else {
            boolean hasTarget = robot.hasValidTarget();
            if (hasTarget) {
                LLResult result = robot.limelight.getLatestResult();
                telemetry.addData("Status", "TARGET LOCKED ✓");
                telemetry.addData("TX", "%.2f", result.getTx());
                telemetry.addData("TY", "%.2f", result.getTy());
            } else {
                telemetry.addData("Status", "NO VISION TARGET");
            }
        }

        telemetry.addLine();
        telemetry.addLine("=== SHOOTER STATUS ===");
        telemetry.addData("Outtake", Outtake ? "ENABLED" : "OFF");

        telemetry.update();
    }

    @Override
    public void stop() {
        robot.setTurretTrackingActive(false);
        super.stop();
    }
}