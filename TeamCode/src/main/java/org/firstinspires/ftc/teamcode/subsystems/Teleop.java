package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.CommandManager;

@TeleOp(name = "Turret CRServo TeleOp")
public class Teleop extends OpMode {
    AllMechs robot;
    Gamepad currentGamepad1, previousGamepad1, currentGamepad2, previousGamepad2;
    private final FtcDashboard dash = FtcDashboard.getInstance();

    @Override
    public void init() {
        robot = new AllMechs(hardwareMap, gamepad1, gamepad2);

        currentGamepad1 = new Gamepad();
        currentGamepad2 = new Gamepad();
        previousGamepad1 = new Gamepad();
        previousGamepad2 = new Gamepad();

        telemetry.addData("Status", "Initialized");
        telemetry.addLine("=== CONTROLS ===");
        telemetry.addData("GP1 R1", "Toggle Turret Tracking");
        telemetry.update();
    }

    @Override
    public void start(){
        // No need for follower methods since we're using direct motor control
    }

    @Override
    public void loop() {
        // Update gamepad states
        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);
        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);

        // ========== MECANUM DRIVE ==========
        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        robot.frontLeft.setPower(frontLeftPower);
        robot.backLeft.setPower(backLeftPower);
        robot.frontRight.setPower(frontRightPower);
        robot.backRight.setPower(backRightPower);

        // Update Limelight
        robot.updateLimelightData();
        boolean hasTarget = robot.hasValidTarget();
        LLResult result = hasTarget ? robot.limelight.getLatestResult() : null;

        // ========== TURRET TRACKING TOGGLE ==========
        if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
            robot.setTurretTrackingActive(!robot.isTurretTrackingActive());
        }

        // Disable tracking if target is lost
//        if (!hasTarget && robot.isTurretTrackingActive()) {
//            robot.setTurretTrackingActive(false);
//        }

        // ========== UPDATE TURRET ==========
        robot.updateTurretTracking();

        // ========== TELEMETRY ==========
        telemetry.addLine("=== LIMELIGHT STATUS ===");
        if (hasTarget && result != null) {
            telemetry.addData("Status", "TARGET LOCKED ✓");
            telemetry.addData("Info", robot.getLimelightTelemetry());
        } else {
            telemetry.addData("Status", "NO TARGET");
        }

        if(gamepad1.dpad_left){
            CommandManager.INSTANCE.scheduleCommand(
                    robot.intakeOn()
            );
        }
        if(gamepad1.dpad_right){
            CommandManager.INSTANCE.scheduleCommand(
                    robot.intakeOff()
            );
        }
        if(gamepad1.dpad_up){
            CommandManager.INSTANCE.scheduleCommand(
                    robot.transferOn()
            );
        }
        if(gamepad1.dpad_down){
            CommandManager.INSTANCE.scheduleCommand(
                    robot.transferOff()
            );
        }
        if(gamepad1.triangle){
            CommandManager.INSTANCE.scheduleCommand(
                    robot.OuttakeOn()
            );
        }
        if(gamepad1.square){
            CommandManager.INSTANCE.scheduleCommand(
                    robot.OuttakeOn()
            );
        }
        if(gamepad1.circle){
            CommandManager.INSTANCE.scheduleCommand(
                    robot.OuttakeOff()
            );
        }
        if(gamepad1.triangle){
            CommandManager.INSTANCE.scheduleCommand(
                    robot.turretOn()
            );
        }
        if(gamepad1.cross){
            CommandManager.INSTANCE.scheduleCommand(
                    robot.OuttakeOff()
            );
        }
        if (gamepad2.dpad_up) {
            CommandManager.INSTANCE.scheduleCommand(
                    robot.doorOpen()
            );
        }
        if (gamepad2.dpad_down) {
            CommandManager.INSTANCE.scheduleCommand(
                    robot.doorClosed()
            );
        }

        CommandManager.INSTANCE.run();

        telemetry.addLine();
        telemetry.addLine("=== TURRET STATUS ===");
        telemetry.addData("Info", robot.getTurretTelemetry());
        telemetry.addData("Control",
                robot.isTurretTrackingActive() ? "AUTO (R1 to disable)" : "READY (R1 to enable)");

        telemetry.update();
    }


    @Override
    public void stop() {
        robot.setTurretTrackingActive(false);
        robot.closeVision();
        super.stop();
    }


}