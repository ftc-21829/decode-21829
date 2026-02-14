package org.firstinspires.ftc.teamcode.blueFar;

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

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.AllMechCopy;
import org.firstinspires.ftc.teamcode.subsystems.PoseStorage;
import org.firstinspires.ftc.teamcode.subsystems.TurretPoseStorage;
import org.firstinspires.ftc.teamcode.testing.DriveTrainFloat;

@Autonomous(name = "Blue15FarCycle", group = "Autonomous")
@Configurable
public class Blue15FarCycle extends OpMode {
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState;
    private AllMechCopy robot;
    private Timer actionTimer;
    private PathChain shoot1Path, pickup1Path, shoot2Path, pickup2Path, shoot3Path, pickup3Path, pickup4Path, shoot4Path, shoot0Path, leavePath, gatePath;
    private boolean shooterActive = false; // ADD THIS


    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(56, 8, Math.toRadians(90)));
        pathState = 0;
        DriveTrainFloat.setToFloatMode(hardwareMap);
        PoseStorage.x = 56;
        PoseStorage.y = 8;
        PoseStorage.heading = Math.toRadians(90);


        robot = new AllMechCopy(hardwareMap, gamepad1, gamepad2, follower);
        TurretPoseStorage.autoEndTurretAngle = robot.getRotation();

        actionTimer = new Timer();

        shoot0Path = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(56, 8, Math.toRadians(90)),
                                new Pose(56, 10.5, Math.toRadians(90))
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();
        // Build the leave path
        pickup1Path = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(56, 10.5, Math.toRadians(90)),
                                new Pose(55.491, 34.406),
                                new Pose(19.096, 35.672, Math.toRadians(180))
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                .build();
        shoot1Path = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(20.096, 35.672, Math.toRadians(180)),
                                new Pose(55.344, 14.090, Math.toRadians(180))
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setVelocityConstraint(10)
                .build();

        pickup2Path = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(55.344, 14.090, Math.toRadians(180)),
                                new Pose(49.878, 9.017),
                                new Pose(14.090, 10.144, Math.toRadians(185))
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(185))
                .build();

        shoot2Path = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(14.090, 10.144, Math.toRadians(185)),
//                                new Pose(83.4021, 87.15887),
                                new Pose(56.078, 13.8082, Math.toRadians(180))

                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(185), Math.toRadians(180))
                .setVelocityConstraint(10)
                .build();
        pickup3Path = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(56.078, 13.8082, Math.toRadians(180)),
                                new Pose(14.090, 10.1448, Math.toRadians(185))

                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(185))
                .setVelocityConstraint(5)
                .build();
        shoot3Path = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(14.090, 10.1448, Math.toRadians(185)),
                                new Pose(56.078, 13.808, Math.toRadians(180))

                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(185), Math.toRadians(180))
                .setVelocityConstraint(10)
                .build();
        pickup4Path = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(56.078, 13.808, Math.toRadians(180)),
                                new Pose(52.133, 25.643),
                                new Pose(13.244, 21.698, Math.toRadians(180))

                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setVelocityConstraint(10)
                .build();
        shoot4Path = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(13.244, 21.698, Math.toRadians(180)),
                                new Pose(54.387, 16.344, Math.toRadians(145))

                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(145))
                .setVelocityConstraint(10)
                .build();
        leavePath = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(54.387, 16.344, Math.toRadians(145)),
                                new Pose(45.088, 23.953, Math.toRadians(145))
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(145))
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
                follower.followPath(shoot0Path);
                robot.UpdateTarget(-8, 156);

                pathState = 1;
                break; // ✅

            case 1:
                // Wait for path to complete AND timer
                if (!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > 4.0) {
                    // Shoot the preload
                    CommandManager.INSTANCE.scheduleCommand(
                            robot.OuttakeOneAuto()
                    );
                    actionTimer.resetTimer();
                    pathState = 2;
                }
                break; // ✅

            case 2:
                // Wait for outtake to complete
                if (actionTimer.getElapsedTimeSeconds() > 3.25) {
                    // Start pickup path
                    follower.followPath(pickup1Path,0.9,  true);

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
                if (actionTimer.getElapsedTimeSeconds() > 2.5) {
                    // Start pickup path
                    follower.followPath(shoot1Path, true);
                    robot.UpdateTarget(0.5,141);

                    actionTimer.resetTimer();
                    pathState = 4;
                }
                break; // ✅
            case 4:
                // Wait for pickup path to complete AND intake to finish
                if (actionTimer.getElapsedTimeSeconds() > 2.45) {
                    CommandManager.INSTANCE.scheduleCommand(
                            robot.OuttakeOne()

                    );

                    actionTimer.resetTimer();

                    pathState = 16;
                }
                break; // ✅

//            case 5:
//                // Wait for path to complete AND shooter to spin up
//                if (!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > 1) {
//                    // Start shoot path
//                    follower.followPath(pickup2Path, true);
//                    CommandManager.INSTANCE.scheduleCommand(
//                            robot.intakeAndTransferAuto()
//                    );
//
//                    actionTimer.resetTimer();
//                    pathState = 6;
//                }
//                break; // ✅
//            case 6:
//                // Wait for outtake to complete
//                if (actionTimer.getElapsedTimeSeconds() > 1.75) {
//                    // Start pickup path
//                    follower.followPath(gatePath, true);
//
//                    actionTimer.resetTimer();
//                    pathState = 7;
//                }
//                break; // ✅
//            case 7:
//                // Wait for pickup path to complete AND intake to finish
//                if (!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > 1.25) {
//                    // Start shoot path
//                    follower.followPath(shoot2Path, true);
//                    robot.UpdateTarget(4,141);
//
//
//                    actionTimer.resetTimer();
//                    pathState = 8;
//                }
//                break; // ✅
//
//            case 8:
//                // Wait for path to complete AND shooter to spin up
//                if (!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > 1.25) {
//                    // Shoot the sample
//                    CommandManager.INSTANCE.scheduleCommand(
//                            robot.OuttakeOne()
//                    );
//
//                    actionTimer.resetTimer();
//                    pathState = 9;
//                }
//                break; // ✅
//            case 9:
//                // Wait for outtake to complete
//                if (actionTimer.getElapsedTimeSeconds() > 1.5) {
//                    // Start pickup path
//                    follower.followPath(pickup3Path, true);
//
//                    // Schedule intake ONCE
//                    CommandManager.INSTANCE.scheduleCommand(
//                            robot.intakeAndTransferAuto()
//                    );
//
//                    actionTimer.resetTimer();
//                    pathState = 10;
//                }
//                break; // ✅
//            case 10:
//                // Wait for pickup path to complete AND intake to finish
//                if (!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > 2.0) {
//                    // Start shoot path
//                    follower.followPath(shoot3Path, true);
//                    robot.UpdateTarget(4, 140);
//
//                    actionTimer.resetTimer();
//                    pathState = 11;
//                }
//                break; // ✅
//
//            case 11:
//                // Wait for path to complete AND shooter to spin up
//                if (!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > 1.5) {
//                    // Shoot the sample
//                    CommandManager.INSTANCE.scheduleCommand(
//                            robot.OuttakeOne()
//                    );
//
//                    actionTimer.resetTimer();
//                    pathState = 12;
//                }
//                break; // ✅
//            case 12:
//                if (actionTimer.getElapsedTimeSeconds() > 1.25) {
//                    follower.followPath(pickup4Path, true);
//                    CommandManager.INSTANCE.scheduleCommand(
//                            robot.intakeAndTransferAuto()
//                    );
//                    pathState = 13; // Done
//                }
//                break; // ✅
//            case 13:
//                // Wait for final path to finish
//                if (!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > 3.15) {
//
//                    follower.followPath(shoot4Path, true);
//                    robot.UpdateTarget(-6, 140);
//                    pathState = 14; // Done
//                }
//                break;
//
//            case 14:
//                if (!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > 2.25) {
//
//                    CommandManager.INSTANCE.scheduleCommand(
//                            robot.OuttakeOne()
//                    );
//                    pathState = 15; // Done
//                }
//                // Autonomous complete, do nothing
//                break;
//            case 15:
//                if (!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > 1.75) {
//
//                    follower.followPath(leavePath);
//                    pathState = 16; // Done
//                }
//                // Autonomous complete, do nothing
//                break;
            case 16:
                // Wait for final path to finish
                if (!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > 2.5) {

                    // Schedule turret zeroing

                    pathState = 17; // Done
                }
                break;

            case 17:
                // Autonomous complete, do nothing
                break;
        }
    }
}


