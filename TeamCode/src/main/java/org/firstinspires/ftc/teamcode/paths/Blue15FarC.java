package org.firstinspires.ftc.teamcode.paths;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.rowanmcalpin.nextftc.core.command.CommandManager;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup;
import com.rowanmcalpin.nextftc.core.command.utility.InstantCommand;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.AllMechCopy;
import org.firstinspires.ftc.teamcode.subsystems.AllMechs;
import org.firstinspires.ftc.teamcode.testing.DriveTrainFloat;

@Autonomous(name = "Blue15FarTest", group = "Autonomous")
@Configurable
public class Blue15FarC extends OpMode {
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState;
    private AllMechCopy robot;
    private Timer actionTimer;
    private PathChain leavePath;

    private boolean shooterActive = false; // ADD THIS

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(56, 8, Math.toRadians(90)));
        pathState = 0;
        DriveTrainFloat.setToFloatMode(hardwareMap);



        robot = new AllMechCopy(hardwareMap, gamepad1, gamepad2, follower);
        actionTimer = new Timer();

        leavePath = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(56, 8, Math.toRadians(90)),
                                new Pose(56, 12, Math.toRadians(90))
                        )
                )
                .build();

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update();


        // Update target based on position
        Pose robotPose = follower.getPose(); // Use follower, not robot.follower
        if(robotPose.getY() < 60){
            robot.UpdateTarget(6, 152);
        } else {
            robot.UpdateTarget(3, 150);
        }

        robot.updateTurretTracking();
        // CRITICAL: Continuously update shooter when active
        if (shooterActive) {
            robot.periodicShooterUpdateAndApplyPID();
        }
        autonomousUpdate();
        CommandManager.INSTANCE.run();

        // Telemetry
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("Shooter Active", shooterActive);
//        panelsTelemetry.debug("X", robot.follower.getPose().getX());
//        panelsTelemetry.debug("Y", robot.follower.getPose().getY());
//        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public void autonomousUpdate() {
        switch (pathState) {
            case 0:
                // Start shooting sequence
                actionTimer.resetTimer();

                // Turn on turret tracking
                robot.setTurretTrackingActive(true);

                // Enable shooter
                shooterActive = true;

                pathState = 1;
                break;

            case 1:
                // Wait for shooter to spin up and turret to align
                if (actionTimer.getElapsedTimeSeconds() > 3.25) {
                    // Kick the sample out
                    CommandManager.INSTANCE.scheduleCommand(robot.OuttakeOne());

                    actionTimer.resetTimer();
                    pathState = 2;
                }
                break;

            case 2:
                // Wait for outtake sequence to complete
                if (actionTimer.getElapsedTimeSeconds() > 2.5) {
                    // Turn off shooter
                    shooterActive = false;

                    // Turn off turret
                    robot.setTurretTrackingActive(false);

                    pathState = 3;
                }
                break;

            case 3:
                // Wait for path to complete
                if (!follower.isBusy()) {
                    pathState = 4; // Done
                }
                break;

            case 4:
                // Autonomous complete - do nothing
                break;
        }
    }
}
