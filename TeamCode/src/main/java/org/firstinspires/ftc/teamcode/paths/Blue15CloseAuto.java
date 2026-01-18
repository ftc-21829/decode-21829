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
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.core.command.utility.InstantCommand;
import com.rowanmcalpin.nextftc.core.command.utility.delays.Delay;
import com.rowanmcalpin.nextftc.core.command.utility.delays.WaitUntil;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.AllMechCopy;
import org.firstinspires.ftc.teamcode.subsystems.AllMechs;
import org.firstinspires.ftc.teamcode.subsystems.PoseStorage;
import org.firstinspires.ftc.teamcode.testing.DriveTrainFloat;

@Autonomous(name = "Blue15Close", group = "Autonomous")
@Configurable
public class Blue15CloseAuto extends OpMode {
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState;
    private AllMechCopy robot;
    private Timer actionTimer;
    private PathChain shoot1Path, pickup1Path, shoot2Path, pickup2Path,shoot3Path, pickup3Path, shoot4Path, leavePath;
    private boolean shooterActive = false; // ADD THIS


    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(20.67, 122.855, Math.toRadians(144)));
        pathState = 0;
        DriveTrainFloat.setToFloatMode(hardwareMap);



        robot = new AllMechCopy(hardwareMap, gamepad1, gamepad2, follower);
        actionTimer = new Timer();

        // Build the leave path
        shoot1Path = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(20.67, 122.855, Math.toRadians(144)),
                                new Pose(55.204, 84.095, Math.toRadians(180))
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180))
                .build();

        pickup1Path = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(55.204, 84.095, Math.toRadians(180)),
                                new Pose(25.37, 84.124, Math.toRadians(180))
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setVelocityConstraint(20)
                .build();
        shoot2Path = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(25.37, 84.124, Math.toRadians(180)),
                                new Pose(55.204, 84.095, Math.toRadians(180))
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        pickup2Path = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(55.204, 84.095, Math.toRadians(180)),
                                new Pose(60.5979, 56.84113),
                                new Pose(22.359, 55.263, Math.toRadians(180))

                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setVelocityConstraint(15)
                .build();
        shoot3Path = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(22.359, 55.263, Math.toRadians(180)),
                                new Pose(43.5636, 62.6470),
                                new Pose(55.204, 84.095, Math.toRadians(180))

                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        pickup3Path = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(55.204, 84.095, Math.toRadians(180)),
                                new Pose(65.0269, 30.0499),
                                new Pose(20.096, 35.672, Math.toRadians(180))

                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setVelocityConstraint(20)
                .build();
        shoot4Path = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(20.096, 35.672, Math.toRadians(180)),
                                new Pose(55.204, 84.095, Math.toRadians(180))
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        leavePath = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(55.204, 84.095, Math.toRadians(180)),
                                new Pose(45, 60.095, Math.toRadians(180))
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update();
        PoseStorage.lastPose = follower.getPose();


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

    //    public void autonomousUpdate() {
//        switch (pathState) {
//            case 0:
//                // Start shooting sequence
//                actionTimer.resetTimer();
//                robot.setTurretTrackingActive(true);
//
//                shooterActive = true;
//                follower.followPath(shoot1Path, false);
//                pathState = 1;
//                break;
//            case 1:
//                // Wait for shooting to complete (adjust time as needed)
//                if (actionTimer.getElapsedTimeSeconds() > 2.55) {
//                    // Kick the sample out
//
//                    CommandManager.INSTANCE.scheduleCommand(
//                            new SequentialGroup(
//                                    robot.OuttakeOne()
//                            ));
//                    if (!follower.isBusy()) {
//                        pathState = 2; // Done
//                        actionTimer.resetTimer();
//                        shooterActive = false;
//                    }
//                }
//                break;
//
//
//            case 2:
//                // Wait for butt kicker to complete
//                if (actionTimer.getElapsedTimeSeconds() > 2) {
//                    follower.followPath(pickup1Path);
//
//                    CommandManager.INSTANCE.scheduleCommand(
//                            robot.intakeAndTransfer()
//                            );
//
//                    if (!follower.isBusy()) {
//                        pathState = 3;
//                        shooterActive = true;
//                        actionTimer.resetTimer();
//                    }
//                }
//                break;
//
//            case 3:
//                if (actionTimer.getElapsedTimeSeconds() > 4) {
//                    follower.followPath(shoot2Path);
//
//                    CommandManager.INSTANCE.scheduleCommand(
//                            robot.OuttakeOne()
//                    );
//
//                    if (!follower.isBusy()) {
//                        pathState = 4;
//                        shooterActive = false;
//                        actionTimer.resetTimer();
//                    }
//                }
//                break;
//            case 4:
//                // Wait for path to complete
//                if (!follower.isBusy()) {
//                    pathState = 5; // Done
//
//                }
//                break;
//
//            case 5:
//                // Autonomous complete - do nothing
//                break;
//        }
//    }
    public void autonomousUpdate() {
        switch (pathState) {
            case 0:
                // Start shooting sequence
                actionTimer.resetTimer();
                robot.setTurretTrackingActive(true);
                shooterActive = true;

                follower.followPath(shoot1Path, false);

                pathState = 1;
                break; // ✅

            case 1:
                // Wait for path to complete AND timer
                if (!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > 2.55) {
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
                if (actionTimer.getElapsedTimeSeconds() > 2.5) {
                    // Start pickup path
                    follower.followPath(pickup1Path, false);

                    // Schedule intake ONCE
                    CommandManager.INSTANCE.scheduleCommand(
                            robot.intakeAndTransfer()
                    );

                    actionTimer.resetTimer();
                    pathState = 3;
                }
                break; // ✅

            case 3:
                // Wait for pickup path to complete AND intake to finish
                if (!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > 2.0) {
                    // Start shoot path
                    follower.followPath(shoot2Path, false);

                    actionTimer.resetTimer();
                    pathState = 4;
                }
                break; // ✅

            case 4:
                // Wait for path to complete AND shooter to spin up
                if (!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > 2.5) {
                    // Shoot the sample
                    CommandManager.INSTANCE.scheduleCommand(
                                    robot.OuttakeOne()

                    );

                    actionTimer.resetTimer();
                    pathState = 5;
                }
                break; // ✅
            case 5:
                // Wait for outtake to complete
                if (actionTimer.getElapsedTimeSeconds() > 2.5) {
                    // Start pickup path
                    follower.followPath(pickup2Path, false);

                    // Schedule intake ONCE
                    CommandManager.INSTANCE.scheduleCommand(
                            robot.intakeAndTransfer()
                    );

                    actionTimer.resetTimer();
                    pathState = 6;
                }
                break; // ✅
            case 6:
                // Wait for pickup path to complete AND intake to finish
                if (!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > 2.25) {
                    // Start shoot path
                    follower.followPath(shoot3Path, true);

                    actionTimer.resetTimer();
                    pathState = 7;
                }
                break; // ✅

            case 7:
                // Wait for path to complete AND shooter to spin up
                if (!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > 2.75) {
                    // Shoot the sample
                    CommandManager.INSTANCE.scheduleCommand(
                            robot.OuttakeOne()

                    );

                    actionTimer.resetTimer();
                    pathState = 8;
                }
                break; // ✅
            case 8:
                // Wait for outtake to complete
                if (actionTimer.getElapsedTimeSeconds() > 2.5) {
                    // Start pickup path
                    follower.followPath(pickup3Path, false);

                    // Schedule intake ONCE
                    CommandManager.INSTANCE.scheduleCommand(
                            robot.intakeAndTransfer()
                    );

                    actionTimer.resetTimer();
                    pathState = 9;
                }
                break; // ✅
            case 9:
                // Wait for pickup path to complete AND intake to finish
                if (!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > 2.0) {
                    // Start shoot path
                    follower.followPath(shoot4Path, true);

                    actionTimer.resetTimer();
                    pathState = 10;
                }
                break; // ✅

            case 10:
                // Wait for path to complete AND shooter to spin up
                if (!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > 3.15) {
                    // Shoot the sample
                    CommandManager.INSTANCE.scheduleCommand(
                            robot.OuttakeOne()

                    );

                    actionTimer.resetTimer();
                    pathState = 11;
                }
                break; // ✅
            case 11:
                if (actionTimer.getElapsedTimeSeconds() > 1.5) {
                    follower.followPath(leavePath);
                    pathState = 12; // Done
                }
                break; // ✅
            case 12:
                // Wait for final outtake to complete
                if (actionTimer.getElapsedTimeSeconds() > 2.5) {
                    pathState = 12; // Done
                }
                break; // ✅

            case 13:
                // Autonomous complete - do nothing
                break;
        }
    }
}



//package org.firstinspires.ftc.teamcode.paths;
//
//import com.bylazar.configurables.annotations.Configurable;
//import com.bylazar.telemetry.PanelsTelemetry;
//import com.bylazar.telemetry.TelemetryManager;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.BezierCurve;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.Path;
//import com.pedropathing.paths.PathChain;
//import com.pedropathing.util.Timer;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.rowanmcalpin.nextftc.core.command.CommandManager;
//import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup;
//import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
//
//import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//import org.firstinspires.ftc.teamcode.subsystems.AllMechCopy;
//
//@Autonomous(name = "Blue15Close", group = "Autonomous")
//@Configurable // Panels
//public class Blue15CloseAuto extends OpMode {
//    private Timer pathTimer, opmodeTimer;
//    private Timer actionTimer;
//    private AllMechCopy robot;
//
//    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
//    public Follower follower; // Pedro Pathing follower instance
//    private int pathState; // Current autonomous path state (state machine)
//
//    private Paths paths; // Paths defined in the Paths class
//
//    @Override
//    public void init() {
//        pathTimer = new Timer();
//        opmodeTimer = new Timer();
//        opmodeTimer.resetTimer();
//        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
//        actionTimer = new Timer();
//        follower = Constants.createFollower(hardwareMap);
//        follower.setStartingPose(new Pose(20.67, 122.855, Math.toRadians(144)));
//
//        paths = new Paths(follower); // Build paths
//
//        panelsTelemetry.debug("Status", "Initialized");
//        panelsTelemetry.update(telemetry);
//    }
//
//    @Override
//    public void loop() {
//        follower.update(); // Update Pedro Pathing
//        autonomousPathUpdate(); // Update autonomous state machine
//
//        // Log values to Panels and Driver Station
//        panelsTelemetry.debug("Path State", pathState);
//        panelsTelemetry.debug("X", follower.getPose().getX());
//        panelsTelemetry.debug("Y", follower.getPose().getY());
//        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
//        panelsTelemetry.update(telemetry);
//    }
//
//    public class Paths {
//
//        public Path ShootingPreload;
//        public PathChain Intaking1;
//        public PathChain Shooting1;
//        public PathChain Intaking2;
//        public PathChain Shooting2;
//        public PathChain Intaking3;
//        public PathChain Outtake3;
//        public PathChain Intake4;
//        public PathChain Outtake4;
//        public PathChain Leave;
//
//        public Paths(Follower follower) {
//
//            ShootingPreload = new Path(new BezierLine(new Pose(20.67, 122.855), new Pose(52.62, 87.384)));
//            ShootingPreload.setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(170));
//
//
//            Intaking1 = follower
//                    .pathBuilder()
//                    .addPath(
//                            new BezierLine(new Pose(49.856+24, 84.331), new Pose(18.139+24, 84.495))
//                    )
//                    .setConstantHeadingInterpolation(Math.toRadians(180))
//                    .build();
//
//            Shooting1 = follower
//                    .pathBuilder()
//                    .addPath(
//                            new BezierLine(new Pose(18.139+24, 84.495), new Pose(51.713+24, 86.453))
//                    )
//                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(130))
//                    .setReversed()
//                    .build();
//
//            Intaking2 = follower
//                    .pathBuilder()
//                    .addPath(
//                            new BezierCurve(
//                                    new Pose(51.713+24, 86.453),
//                                    new Pose(49.328+24, 43.795),
//                                    new Pose(14.320+24, 64.707)
//                            )
//                    )
//                    .setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(180))
//                    .build();
//
//            Shooting2 = follower
//                    .pathBuilder()
//                    .addPath(
//                            new BezierCurve(
//                                    new Pose(14.320+24, 64.707),
//                                    new Pose(43.757+24, 71.072),
//                                    new Pose(50.917+24, 84.066)
//                            )
//                    )
//                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
//                    .build();
//
//            Intaking3 = follower
//                    .pathBuilder()
//                    .addPath(
//                            new BezierCurve(
//                                    new Pose(50.917+24, 84.066),
//                                    new Pose(59.724+24, 28.753),
//                                    new Pose(18.033+24, 34.740)
//                            )
//                    )
//                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
//                    .build();
//
//            Outtake3 = follower
//                    .pathBuilder()
//                    .addPath(
//                            new BezierLine(new Pose(18.033+24, 34.740), new Pose(50.917+24, 84.066))
//                    )
//                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
//                    .build();
//
//            Intake4 = follower
//                    .pathBuilder()
//                    .addPath(
//                            new BezierCurve(
//                                    new Pose(50.917+24, 84.066),
//                                    new Pose(12.729+24, 55.160),
//                                    new Pose(8.406+24, 9.067)
//                            )
//                    )
//                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(270))
//                    .build();
//
//            Outtake4 = follower
//                    .pathBuilder()
//                    .addPath(
//                            new BezierCurve(
//                                    new Pose(8.406+24, 9.067),
//                                    new Pose(39.374+24, 16.808),
//                                    new Pose(63.927+24, 15.039)
//                            )
//                    )
//                    .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(125))
//                    .build();
//
//            Leave = follower
//                    .pathBuilder()
//                    .addPath(
//                            new BezierLine(new Pose(63.927+24, 15.039), new Pose(63.706+24, 39.592))
//                    )
//                    .setLinearHeadingInterpolation(Math.toRadians(125), Math.toRadians(90))
//                    .build();
//        }
//    }
//
//    public void autonomousPathUpdate() {
//        switch (pathState) {
//            case 0:
//                // Start shooting sequence
//                actionTimer.resetTimer();
//                CommandManager.INSTANCE.scheduleCommand(
//                        new ParallelGroup(
//                                robot.OuttakeOn()),
//                                robot.turret()
//
//
//                );
//
//                pathState = 1;
//                break;
//
//            case 1:
//                // Wait for shooting to complete (adjust time as needed)
//                if (actionTimer.getElapsedTimeSeconds() > 3.25) {
//                    // Kick the sample out
//
//                    CommandManager.INSTANCE.scheduleCommand(
//                            new SequentialGroup(
//                                    robot.OuttakeOne()
//                            ));
//                    if (!follower.isBusy()) {
//                        pathState = 2; // Done
//                        actionTimer.resetTimer();
//                    }
//                    break;
//
//                }
//
//
//            case 2:
//                // Wait for butt kicker to complete
//                if (actionTimer.getElapsedTimeSeconds() > 2) {
////
////                    // Turn off mechanisms
////                    CommandManager.INSTANCE.scheduleCommand(
////
////                                    robot.ButtKicker()
////
////                            );
////                    actionTimer.resetTimer();
//////                     Start leaving
////                    pathState = 3;
////                }
////                break;
////
////            case 3:
////                // Wait for butt kicker to complete
////                if (actionTimer.getElapsedTimeSeconds() > 5) {
////                    // Turn off mechanisms
////                    CommandManager.INSTANCE.scheduleCommand(
////                            new ParallelGroup(
////                                    robot.transferOff(),
////                                    robot.OuttakeOff(),
////                                    robot.doorClose(),
////                                    robot.intakeOff()
////                            ));
////                     Start leaving
//                    follower.followPath();
//                    pathState = 3;
//                }
//                break;
//            case 3:
//                // Wait for path to complete
//                if (!follower.isBusy()) {
//                    pathState = 4; // Done
//
//                }
//                break;
//
//            case 4:
//                // Autonomous complete - do nothing
//                break;
//        }
//    }
////        switch (pathState) {
////            case 0:
////                follower.followPath(paths.ShootingPreload);
////                setPathState(1);
////                break;
////            case 1:
////            /* You could check for
////            - Follower State: "if(!follower.isBusy()) {}"
////            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
////            - Robot Position: "if(follower.getPose().getX() > 36) {}"
////            */
////
////                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
////                if(!follower.isBusy()) {
////                    /* Score Preload */
////
////                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
////                    follower.followPath(paths.Intaking1,true);
////                    setPathState(2);
////                }
////                break;
////            case 2:
////                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
////                if(!follower.isBusy()) {
////                    /* Grab Sample */
////
////                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
////                    follower.followPath(paths.Shooting1,true);
////                    setPathState(3);
////                }
////                break;
////            case 3:
////                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
////                if(!follower.isBusy()) {
////                    /* Score Sample */
////
////                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
////                    follower.followPath(paths.Intaking2,true);
////                    setPathState(4);
////                }
////                break;
////            case 4:
////                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
////                if(!follower.isBusy()) {
////                    /* Grab Sample */
////
////                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
////                    follower.followPath(paths.Shooting2,true);
////                    setPathState(5);
////                }
////                break;
////            case 5:
////
////                if(!follower.isBusy()) {
////                    follower.followPath(paths.Intaking3, true);
////                    setPathState(6);
////
////                }
////                break;
////            case 6:
////
////                if(!follower.isBusy()) {
////                    follower.followPath(paths.Outtake3, true);
////                    setPathState(7);
////                }
////                break;
////            case 7:
////
////                if(!follower.isBusy()) {
////                    follower.followPath(paths.Intake4, true);
////                    setPathState(8);
////                }
////                break;
////            case 8:
////
////                if(!follower.isBusy()) {
////                    follower.followPath(paths.Outtake4, true);
////                    setPathState(9);
////                }
////                break;
////            case 9:
////
////                if(!follower.isBusy()) {
////                    follower.followPath(paths.Leave, true);
////                    setPathState(10);
////                }
////                break;
////            case 10:
////                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
////                if(!follower.isBusy()) {
////                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
////                    setPathState(-1);
////                }
////                break;
////        }
//    }
//
//    public void setPathState(int pState) {
//        pathState = pState;
//        pathTimer.resetTimer();
//    }
//}