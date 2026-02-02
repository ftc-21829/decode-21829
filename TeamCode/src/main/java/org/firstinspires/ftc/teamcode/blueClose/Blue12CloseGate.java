package org.firstinspires.ftc.teamcode.blueClose;

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
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.core.command.utility.delays.Delay;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.AllMechCopy;
import org.firstinspires.ftc.teamcode.subsystems.PoseStorage;
import org.firstinspires.ftc.teamcode.subsystems.TurretPoseStorage;
import org.firstinspires.ftc.teamcode.testing.DriveTrainFloat;

@Autonomous(name = "Blue12CloseGate", group = "Autonomous")
@Configurable
public class Blue12CloseGate extends OpMode {
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState;
    private AllMechCopy robot;
    private Timer actionTimer;
    private PathChain shoot1Path, pickup1Path, shoot2Path, pickup2Path, shoot3Path, pickup3Path, shoot4Path, leavePath, gatePath;
    private boolean shooterActive = false; // ADD THIS


    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(20.67, 122.855, Math.toRadians(144)));
        pathState = 0;
        DriveTrainFloat.setToFloatMode(hardwareMap);
        PoseStorage.x = 20.67;
        PoseStorage.y = 122.855;
        PoseStorage.heading = Math.toRadians(144);


        robot = new AllMechCopy(hardwareMap, gamepad1, gamepad2, follower);

        TurretPoseStorage.autoEndTurretAngle = robot.getRotation();

        actionTimer = new Timer();

        // Build the leave path
        shoot1Path = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(20.67, 122.855, Math.toRadians(144)),
                                new Pose(53.704, 85.595, Math.toRadians(180))
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180))
                .build();

        pickup1Path = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(53.704, 85.595, Math.toRadians(180)),
                                new Pose(26.37, 86.124, Math.toRadians(180))
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .setVelocityConstraint(10)
                .build();
        gatePath = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(26.37, 86.124, Math.toRadians(180)),
                                new Pose(38.38157422512238, 80.09136378466565),
                                new Pose(16.309298531810772, 75.12495921696577, Math.toRadians(180))
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .setVelocityConstraint(5)
                .build();
        shoot2Path = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(16.309298531810772, 75.12495921696577, Math.toRadians(180)),
                                new Pose(53.704, 85.595, Math.toRadians(144))
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(144))
                .build();
        pickup2Path = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(53.704, 85.595, Math.toRadians(144)),
                                new Pose(60.5979, 56.84113),
                                new Pose(22.859, 55.863, Math.toRadians(180))

                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180))
                .setVelocityConstraint(10)
                .build();
        shoot3Path = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(22.359, 55.863, Math.toRadians(180)),
                                new Pose(43.5636, 62.6470),
                                new Pose(53.704, 85.595, Math.toRadians(144))

                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(144))
                .build();
        pickup3Path = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(53.704, 85.595, Math.toRadians(144)),
                                new Pose(65.0269, 30.0499),
                                new Pose(19.096, 35.672, Math.toRadians(180))

                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180))
                .setVelocityConstraint(10)
                .build();
        shoot4Path = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(19.096, 35.672, Math.toRadians(180)),
                                new Pose(53.704, 85.595, Math.toRadians(144))
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(144))
                .build();
        leavePath = follower
                .pathBuilder()
                .addPath(

                        new BezierLine(
                                new Pose (53.704, 85.595, Math.toRadians(144)),
                                new Pose (48,72, Math.toRadians(144))
                        )

                )
                .setConstantHeadingInterpolation(Math.toRadians(144))
                .build();



        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update();

        PoseStorage.x = follower.getPose().getX();
        PoseStorage.y = follower.getPose().getY();
        PoseStorage.heading = follower.getPose().getHeading();
        TurretPoseStorage.autoEndTurretAngle= (robot.axon.getTotalRotation()/2.37931024483);


        Pose robotPose = follower.getPose(); // Use follower, not robot.follower


        robot.updateTurretTracking_NoTurretRel();
        // CRITICAL: Continuously update shooter when active
        if (shooterActive) {
            robot.periodicShooterUpdateAndApplyPID();
        } else {
            robot.OuttakeOff();
        }
        // Telemetry
        panelsTelemetry.debug("Path State", pathState);
//        panelsTelemetry.debug("X", robot.follower.getPose().getX());
//        panelsTelemetry.debug("Y", robot.follower.getPose().getY());
//        panelsTelemetry.debug("Heading", robot.follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);

        autonomousUpdate();
        CommandManager.INSTANCE.run();
    }
    public void autonomousUpdate() {
        switch (pathState) {
            case 0:
                // Start shooting sequence
                actionTimer.resetTimer();
                robot.setTurretTrackingActive(true);
                shooterActive = true;
                robot.UpdateTarget(-3,150);

                follower.followPath(shoot1Path, false);

                pathState = 1;
                break; // ✅

            case 1:
                // Wait for path to complete AND timer
                if (!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > 2.25) {
                    // Shoot the preload
                    CommandManager.INSTANCE.scheduleCommand(
                            robot.OuttakeOne()

                    );
                    actionTimer.resetTimer();
                    pathState = 2;
                }
                break; // ✅

            case 2:
                // Wait for outtake to complete
                if (actionTimer.getElapsedTimeSeconds() > 2.25) {
                    // Start pickup path
                    follower.followPath(pickup1Path, false);
                    robot.UpdateTarget(-9,135.5);

                    // Schedule intake ONCE
                    CommandManager.INSTANCE.scheduleCommand(
                            robot.intakeAndTransferAuto()
                    );

                    actionTimer.resetTimer();
                    pathState = 3;
                }
                break; // ✅
            case 3:
                // Wait for outtake to complete
                if (actionTimer.getElapsedTimeSeconds() > 2.0) {
                    // Start pickup path
                    follower.followPath(gatePath, true);

                    // Schedule intake ONCE


                    actionTimer.resetTimer();
                    pathState = 4;
                }
                break; // ✅
            case 4:
                // Wait for pickup path to complete AND intake to finish
                if (!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > 2.75) {
                    // Start shoot path
                    follower.followPath(shoot2Path, false);

                    actionTimer.resetTimer();
                    pathState = 5;
                }
                break; // ✅


            case 5:
                // Wait for path to complete AND shooter to spin up
                if (!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > 2.15) {
                    // Shoot the sample
                    CommandManager.INSTANCE.scheduleCommand(
                            robot.OuttakeOne()

                    );

                    actionTimer.resetTimer();
                    pathState = 6;
                }
                break; // ✅
            case 6:
                // Wait for outtake to complete
                if (actionTimer.getElapsedTimeSeconds() > 2.1) {
                    // Start pickup path
                    follower.followPath(pickup2Path, false);

                    // Schedule intake ONCE
                    CommandManager.INSTANCE.scheduleCommand(
                            robot.intakeAndTransferAuto()
                    );

                    actionTimer.resetTimer();
                    pathState = 7;
                }
                break; // ✅
            case 7:
                // Wait for pickup path to complete AND intake to finish
                if (!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > 2.75) {
                    // Start shoot path
                    follower.followPath(shoot3Path, false);
                    robot.UpdateTarget(-10.5, 133.5);

                    actionTimer.resetTimer();
                    pathState = 8;
                }
                break; // ✅

            case 8:
                // Wait for path to complete AND shooter to spin up
                if (!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > 2.45) {
                    // Shoot the sample
                    CommandManager.INSTANCE.scheduleCommand(
                            robot.OuttakeOne()

                    );

                    actionTimer.resetTimer();
                    pathState = 9;
                }
                break; // ✅
            case 9:
                // Wait for outtake to complete
                if (actionTimer.getElapsedTimeSeconds() > 2.5) {
                    // Start pickup path
                    follower.followPath(pickup3Path, false);
                    robot.UpdateTarget(-12,134);

                    // Schedule intake ONCE
                    CommandManager.INSTANCE.scheduleCommand(
                            robot.intakeAndTransferAuto()
                    );

                    actionTimer.resetTimer();
                    pathState = 10;
                }
                break; // ✅
            case 10:
                // Wait for pickup path to complete AND intake to finish
                if (!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > 2.75) {
                    // Start shoot path
                    follower.followPath(shoot4Path, false);


                    actionTimer.resetTimer();
                    pathState = 11;
                }
                break; // ✅

            case 11:
                // Wait for path to complete AND shooter to spin up
                if (!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > 2.45) {
                    // Shoot the sample
                    CommandManager.INSTANCE.scheduleCommand(
                            new SequentialGroup(
                                    robot.OuttakeOne()

                            )

                    );

                    actionTimer.resetTimer();
                    pathState = 12;
                }
                break; // ✅
            case 12:
                if (actionTimer.getElapsedTimeSeconds() > 2.5) {
                    follower.followPath(leavePath);
                    pathState = 13; // Done
                }
                break; // ✅
            case 13:
                // Wait for final path to finish
                if (!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > 2.5) {

                    // Schedule turret zeroing

                    pathState = 14; // Done
                }
                break;

            case 14:
                // Autonomous complete, do nothing
                break;
        }



    }
}