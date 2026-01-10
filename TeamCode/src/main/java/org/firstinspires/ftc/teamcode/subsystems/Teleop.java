package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.CommandManager;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup;
import com.rowanmcalpin.nextftc.core.command.utility.InstantCommand;
import com.rowanmcalpin.nextftc.core.command.utility.delays.WaitUntil;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "Turret CRServo TeleOp - Blue")
public class Teleop extends OpMode {
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
        telemetry.addData("GP1 R1", "Toggle Turret Tracking");
        telemetry.update();
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

        double y = -gamepad2.left_stick_y;
        double x = -gamepad2.left_stick_x * 1.1;
        double rx = -gamepad2.right_stick_x;

        robot.follower.setTeleOpDrive(y, x, rx, true);

        // Update Limelight
        robot.updateLimelightData();
        boolean hasTarget = robot.hasValidTarget();
        LLResult result = hasTarget ? robot.limelight.getLatestResult() : null;


        // ========== UPDATE TURRET ==========
        robot.updateTurretTracking();

        // ========== TELEMETRY ==========
        telemetry.addLine("=== LIMELIGHT STATUS ===");
        if (hasTarget && result != null) {
            telemetry.addData("Status", "TARGET LOCKED ✓");

        } else {
            telemetry.addData("Status", "NO TARGET");
        }

        if(gamepad1.dpad_right){
            CommandManager.INSTANCE.scheduleCommand(
                    robot.intakeOn()
            );
        }
        if(gamepad1.dpad_left){
            CommandManager.INSTANCE.scheduleCommand(
                    robot.intakeOff()
            );
        }
        if(gamepad1.dpad_up){
            CommandManager.INSTANCE.scheduleCommand(
                    new ParallelGroup(
                            robot.transferOn(),
                            robot.intakeOn(),
                            robot.transferCheck(),
                            robot.doorOpen(),
                            new WaitUntil(()->{
                                double current = robot.intake.getCurrent(CurrentUnit.MILLIAMPS);
                                return current> (-1093.8*robot.battery.getVoltage()*robot.battery.getVoltage())+(27011*robot.battery.getVoltage())-160045;
                            }).then(
                                    robot.intakeOff()
                            )
                    )
            );
        }
        if (gamepad1.right_bumper){

            CommandManager.INSTANCE.scheduleCommand(
                    new ParallelGroup(
                            robot.transferOn(),
                            robot.doorOpen()
                    )
            );

        }
        if(gamepad1.left_bumper){
            CommandManager.INSTANCE.scheduleCommand(
                    robot.transferOff()
            );
        }
        if(gamepad1.triangle){
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


        if(gamepad1.circle){
            Outtake=false;
//
        }
        if(gamepad2.square){
            CommandManager.INSTANCE.scheduleCommand(
                    robot.turretOn()
            );
        }

        if(gamepad2.cross){
            CommandManager.INSTANCE.scheduleCommand(
                    robot.turret()
            );
        }
        if(gamepad2.circle){
            CommandManager.INSTANCE.scheduleCommand(
                    robot.OuttakeOne()
            );
        }
        if(gamepad1.dpad_down){
            CommandManager.INSTANCE.scheduleCommand(
                    robot.IntakeOut()
            );
        }




        if(gamepad1.right_stick_button) {
            CommandManager.INSTANCE.scheduleCommand(
                    robot.ButtKicker()
            );
        }


        CommandManager.INSTANCE.run();




        telemetry.addLine();

        telemetry.addLine("=== TURRET STATUS ===");
        telemetry.addData("Control",
                robot.isTurretTrackingActive() ? "AUTO (R1 to disable)" : "READY (R1 to enable)");
        telemetry.addData("Current", robot.intake.getCurrent(CurrentUnit.MILLIAMPS));
        telemetry.addData("Field Active: ", robot.FieldRelativeTrue());


        telemetry.update();
    }


    @Override
    public void stop() {
        robot.setTurretTrackingActive(false);
        super.stop();
    }


}
