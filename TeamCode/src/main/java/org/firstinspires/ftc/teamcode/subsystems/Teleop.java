package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.CommandManager;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup;
import com.rowanmcalpin.nextftc.core.command.utility.InstantCommand;
import com.rowanmcalpin.nextftc.core.command.utility.delays.WaitUntil;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.testing.DriveTrainFloat;

@TeleOp(name = "Turret CRServo TeleOp - Blue")
public class Teleop extends OpMode {
    AllMechCopy robot;
    boolean Outtake;
    Gamepad currentGamepad1, previousGamepad1, currentGamepad2, previousGamepad2;
    private final FtcDashboard dash = FtcDashboard.getInstance();
    private int lastLowPos = 0;
    private int lastHighPos = 0;
    private long lastTime = 0;
    private double currentVelocity = 0.0;
    private double manualXOffset = 0.0;
    private double manualYOffset = 0.0;
    public static double ADJUSTMENT_STEP = 0.5;

    @Override
    public void init() {
        Follower follower = Constants.createFollower(hardwareMap);
        robot = new AllMechCopy(hardwareMap, gamepad1, gamepad2, follower);

        DriveTrainFloat.setToFloatMode(hardwareMap);

        currentGamepad1 = new Gamepad();
        currentGamepad2 = new Gamepad();
        previousGamepad1 = new Gamepad();
        previousGamepad2 = new Gamepad();

        telemetry.addData("Status", "Initialized");
        telemetry.addLine("=== CONTROLS ===");
        telemetry.addData("GP1 R1", "Toggle Turret Tracking");
        telemetry.update();
        robot.follower.setStartingPose(new Pose(72, 72, Math.toRadians(90)));

        lastLowPos = robot.outtakeLow.getCurrentPosition();
        lastHighPos = robot.outtakeHigh.getCurrentPosition();
        lastTime = System.nanoTime();
    }

    @Override
    public void start(){
        robot.follower.startTeleopDrive();
    }

    @Override
    public void loop() {

        // Update gamepad states
        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);
        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);
        robot.follower.update();
//s
        calculateShooterVelocity();

        double y = -gamepad2.left_stick_y;
        double x = -gamepad2.left_stick_x * 1.1;
        double rx = -gamepad2.right_stick_x;

        robot.follower.setTeleOpDrive(y, x, rx, true);

        // Update Limelight
        robot.updateLimelightData();
        boolean hasTarget = robot.hasValidTarget();
        LLResult result = hasTarget ? robot.limelight.getLatestResult() : null;
        Pose robotPose = robot.follower.getPose();


        // ========== UPDATE TURRET ==========
        robot.updateTurretTracking();

        // ========== TELEMETRY ==========
        telemetry.addLine("=== LIMELIGHT STATUS ===");
        if (hasTarget && result != null) {
            telemetry.addData("Status", "TARGET LOCKED ✓");

        } else {
            telemetry.addData("Status", "NO TARGET");
        }

        if(gamepad1.dpadRightWasPressed()){
            CommandManager.INSTANCE.scheduleCommand(
                    robot.intakeOn()
            );
        }
        if(gamepad1.dpadLeftWasPressed()){
            CommandManager.INSTANCE.scheduleCommand(
                    robot.intakeOff()
            );
        }
        if(gamepad1.dpadUpWasPressed()){
            CommandManager.INSTANCE.scheduleCommand(
                    robot.intakeAndTransfer()
            );
        }
        if (gamepad1.rightBumperWasPressed()){

            CommandManager.INSTANCE.scheduleCommand(
                    new ParallelGroup(
                            robot.transferOn(),
                            robot.doorOpen()
                    )
            );

        }
        if(gamepad1.leftBumperWasPressed()){
            CommandManager.INSTANCE.scheduleCommand(
                    robot.transferOff()
            );
        }
        if(gamepad1.triangleWasPressed()){
            Outtake = true;

        }


        if (Outtake){
        robot.periodicShooterUpdateAndApplyPID();

        }
        if (!Outtake){
            CommandManager.INSTANCE.scheduleCommand(
                   robot.OuttakeOff()
            );
        }

        if(gamepad1.circleWasPressed()){
            Outtake=false;
//
        }
        if(gamepad2.squareWasPressed()){
            CommandManager.INSTANCE.scheduleCommand(
                    robot.turretOn()
            );
        }

        if(gamepad2.crossWasPressed()){
            CommandManager.INSTANCE.scheduleCommand(
                    robot.turret()
            );
        }
        if(gamepad2.circleWasPressed()){
            CommandManager.INSTANCE.scheduleCommand(
                    robot.OuttakeOne()
            );
        }

        if(robotPose.getY()<60){
            robot.UpdateTarget(5.5,152);
        } else {
            robot.UpdateTarget(3.5,148); // 0,148
        }



        if(gamepad1.dpadDownWasPressed()){
            CommandManager.INSTANCE.scheduleCommand(
                    robot.IntakeOut()
            );
        }




        if(gamepad1.leftStickButtonWasPressed()) {
            CommandManager.INSTANCE.scheduleCommand(
                    robot.ButtKicker()
            );
        }


        CommandManager.INSTANCE.run();
        double distanceCmFront = 100;
        double distanceCmBack = 100;
        if(robot.colorSensorFront instanceof DistanceSensor){
            distanceCmFront = ((DistanceSensor) robot.colorSensorFront).getDistance(DistanceUnit.CM);}
        if(robot.colorSensorBack instanceof DistanceSensor){
            distanceCmBack = ((DistanceSensor) robot.colorSensorBack).getDistance(DistanceUnit.CM);}


        telemetry.addLine();

        telemetry.addLine("=== TURRET STATUS ===");
        telemetry.addData("Control",
                robot.isTurretTrackingActive() ? "AUTO (R1 to disable)" : "READY (R1 to enable)");
        telemetry.addData("Current", robot.intake.getCurrent(CurrentUnit.MILLIAMPS));
        telemetry.addData("Field Active: ", robot.FieldRelativeTrue());
        telemetry.addData("Target Velocity", "%.1f ticks/sec", robot.shooterTargetVelocity);
        telemetry.addData("Current Velocity", "%.1f ticks/sec", currentVelocity);
        telemetry.addData("Velocity Error", "%.1f ticks/sec",
                robot.shooterTargetVelocity - currentVelocity);
        telemetry.addData("Front Distance", "%.2f cm", distanceCmFront);
        telemetry.addData("Back Distance", "%.2f cm", distanceCmBack);
        telemetry.addData("Front Ball Detected", distanceCmFront < 6.5 ? "YES ✓" : "NO");
        telemetry.addData("Back Ball Detected", distanceCmBack < 6.5 ? "YES ✓" : "NO");
        telemetry.update();
    }


    @Override
    public void stop() {
        robot.setTurretTrackingActive(false);
        super.stop();
    }

    private void calculateShooterVelocity() {
        int lowPos = robot.outtakeLow.getCurrentPosition();
        int highPos = robot.outtakeHigh.getCurrentPosition();
        long now = System.nanoTime();

        double dt = (now - lastTime) / 1e9;

        if (dt > 0) {
            double lowVel = (lowPos - lastLowPos) / dt;
            double highVel = (highPos - lastHighPos) / dt;
            currentVelocity = (lowVel + highVel) / 2.0;
        }

        lastLowPos = lowPos;
        lastHighPos = highPos;
        lastTime = now;
    }


}
