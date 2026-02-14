package org.firstinspires.ftc.teamcode.redClose;

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

@Autonomous(name = "Red12CloseGate", group = "Autonomous")
@Configurable
public class Red12CloseGate extends OpMode {
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
        follower.setStartingPose(new Pose(123.33, 122.855, Math.toRadians(36)));
        pathState = 0;
        DriveTrainFloat.setToFloatMode(hardwareMap);
        PoseStorage.x = 123.33;
        PoseStorage.y = 122.855;
        PoseStorage.heading = Math.toRadians(36);
        follower.setMaxPower(1);


        robot = new AllMechCopy(hardwareMap, gamepad1, gamepad2, follower);

        TurretPoseStorage.autoEndTurretAngle = robot.getRotation();

        actionTimer = new Timer();

        // Build the leave path
        shoot1Path = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(123.33, 122.855, Math.toRadians(36)),
                                new Pose(90.296, 85.595, Math.toRadians(25))
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(25))
                .build();

        pickup1Path = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(90.296, 85.595, Math.toRadians(25)),
                                new Pose(120.13, 83.124, Math.toRadians(0))
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(25), Math.toRadians(0))
                .build();
        gatePath = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(120.13, 83.124, Math.toRadians(0)),
                                new Pose(105.6185, 80.09136378466565),
                                new Pose(128.69071, 73.12495921696577, Math.toRadians(0))
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        shoot2Path = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(128.69071, 73.12495921696577, Math.toRadians(0)),
                                new Pose(90.296, 85.595, Math.toRadians(36))
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(36))
                .build();
        pickup2Path = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(90.296, 85.595, Math.toRadians(36)),
                                new Pose(83.4021, 56.84113),
                                new Pose(120.441, 56.363, Math.toRadians(0))

                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(0))
                .setVelocityConstraint(5)
                .build();
        shoot3Path = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(120.441, 56.363, Math.toRadians(0)),
                                new Pose(100.4364, 62.6470),
                                new Pose(90.296, 85.595, Math.toRadians(36))

                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(36))
                .build();
        pickup3Path = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(90.296, 85.595, Math.toRadians(36)),
                                new Pose(78.9731, 30.0499),
                                new Pose(124.904, 35.672, Math.toRadians(0))

                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(0))
                .setVelocityConstraint(10)
                .build();
        shoot4Path = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(124.904, 35.672, Math.toRadians(0)),
                                new Pose(90.296, 85.595, Math.toRadians(36))
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(36))
                .build();
        leavePath = follower
                .pathBuilder()
                .addPath(

                        new BezierLine(
                                new Pose (90.296, 85.595, Math.toRadians(36)),
                                new Pose (96,72, Math.toRadians(36))
                        )

                )
                .setConstantHeadingInterpolation(Math.toRadians(36))
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
            robot.periodicShooterUpdateAndApplyPIDRedAuto();
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
                robot.UpdateTarget(134,144);

                follower.followPath(shoot1Path, false);

                pathState = 1;
                break; // ✅

            case 1:
                // Wait for path to complete AND timer
                if (!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > 2) {
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
                if (actionTimer.getElapsedTimeSeconds() > 2.6) {
                    follower.followPath(pickup1Path,0.7, false);
                    robot.UpdateTarget(144.5,140);
//                    robot.UpdateTarget(2,148);


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
                    follower.followPath(gatePath,0.8, true);

                    // Schedule intake ONCE


                    actionTimer.resetTimer();
                    pathState = 4;
                }
                break; // ✅
            case 4:
                // Wait for pickup path to complete AND intake to finish
                if (!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > 2.5) {
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
                if (actionTimer.getElapsedTimeSeconds() > 2.5) {
                    // Start pickup path
                    follower.followPath(pickup2Path, 0.9, false);
                    robot.UpdateTarget(153,137);

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
                if (!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > 2.5) {
                    // Start shoot path
                    follower.followPath(shoot3Path, false);
                    //                    robot.UpdateTarget(1,148);


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
                    robot.UpdateTarget(150,136.5);
//                    robot.UpdateTarget(2,150);


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
                if (!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > 3.25) {
                    // Start shoot path
                    follower.followPath(shoot4Path, false);


                    actionTimer.resetTimer();
                    pathState = 11;
                }
                break; // ✅

            case 11:
                // Wait for path to complete AND shooter to spin up
                if (!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > 2.65) {
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