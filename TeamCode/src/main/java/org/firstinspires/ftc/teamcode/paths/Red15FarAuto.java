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

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.AllMechs;

@Autonomous(name = "Red15Far", group = "Autonomous")
@Configurable
public class Red15FarAuto extends OpMode {
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState;
    private AllMechs robot;
    private Timer actionTimer;
    private PathChain leavePath;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(88, 8, Math.toRadians(90)));
        pathState = 0;


        robot = new AllMechs(hardwareMap, gamepad1, gamepad2);
        actionTimer = new Timer();

        // Build the leave path
        leavePath = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(88, 8, Math.toRadians(90)),
//
                                new Pose(88.327
                                        , 15.294, Math.toRadians(90))
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousUpdate();
        CommandManager.INSTANCE.run();

        // Telemetry
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public void autonomousUpdate() {
        switch (pathState) {
            case 0:
                // Start shooting sequence
                actionTimer.resetTimer();
                CommandManager.INSTANCE.scheduleCommand(
                        new ParallelGroup(
                                robot.OuttakeOn())


                );

                pathState = 1;
                break;

            case 1:
                // Wait for shooting to complete (adjust time as needed)
                if (actionTimer.getElapsedTimeSeconds() > 3.25) {
                    // Kick the sample out

                    CommandManager.INSTANCE.scheduleCommand(
                            new SequentialGroup(
                                    robot.OuttakeOne()
                            ));
                    if (!follower.isBusy()) {
                        pathState = 2; // Done
                        actionTimer.resetTimer();
                    }
                    break;

                }


            case 2:
                // Wait for butt kicker to complete
                if (actionTimer.getElapsedTimeSeconds() > 2) {
//
//                    // Turn off mechanisms
//                    CommandManager.INSTANCE.scheduleCommand(
//
//                                    robot.ButtKicker()
//
//                            );
//                    actionTimer.resetTimer();
////                     Start leaving
//                    pathState = 3;
//                }
//                break;
//
//            case 3:
//                // Wait for butt kicker to complete
//                if (actionTimer.getElapsedTimeSeconds() > 5) {
//                    // Turn off mechanisms
//                    CommandManager.INSTANCE.scheduleCommand(
//                            new ParallelGroup(
//                                    robot.transferOff(),
//                                    robot.OuttakeOff(),
//                                    robot.doorClose(),
//                                    robot.intakeOff()
//                            ));
//                     Start leaving
                    follower.followPath(leavePath);
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

//            case 0:
//                // Start shooting sequence
//                actionTimer.resetTimer();
//                CommandManager.INSTANCE.scheduleCommand(
//                        new ParallelGroup(
//                                robot.OuttakeOn(),
//                                robot.turretOn()
//                        )
//
//
//                );
//
//                pathState = 1;
//                break;
//
//            case 1:
//                // Wait for shooting to complete (adjust time as needed)
//                if (actionTimer.getElapsedTimeSeconds() > 4.5) {
//                    // Kick the sample out
//
//                    CommandManager.INSTANCE.scheduleCommand(
//                            new SequentialGroup(
//                                    robot.transferSlow(),
//                                    robot.doorOpen(),
//                                    robot.intakeOn()
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
//
//                    // Turn off mechanisms
//                    CommandManager.INSTANCE.scheduleCommand(
//
//                            robot.ButtKicker()
//
//                    );
//                    actionTimer.resetTimer();
////                     Start leaving
//                    pathState = 3;
//                }
//                break;
//
//            case 3:
//                // Wait for butt kicker to complete
//                if (actionTimer.getElapsedTimeSeconds() > 5) {
//                    // Turn off mechanisms
//                    CommandManager.INSTANCE.scheduleCommand(
//                            new ParallelGroup(
//                                    robot.transferOff(),
//                                    robot.OuttakeOff(),
//                                    robot.doorClose(),
//                                    robot.intakeOff()
//                            ));
////                     Start leaving
//                    follower.followPath(leavePath);
//                    pathState = 4;
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
//
//    }






//package org.firstinspires.ftc.teamcode.paths;
//
//import com.bylazar.configurables.annotations.Configurable;
//import com.bylazar.telemetry.PanelsTelemetry;
//import com.bylazar.telemetry.TelemetryManager;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.BezierCurve;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.PathChain;
//import com.pedropathing.util.Timer;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.rowanmcalpin.nextftc.core.command.CommandManager;
//import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup;
//import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
//import com.rowanmcalpin.nextftc.core.command.utility.delays.Delay;
//
//import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//import org.firstinspires.ftc.teamcode.subsystems.AllMechs;
//
//@Autonomous(name = "Blue15Farautotestthing", group = "Autonomous")
//@Configurable
//public class Blue15FarAutoFirst extends OpMode {
//    private TelemetryManager panelsTelemetry;
//    public Follower follower;
//    private AllMechs robot;
//    private Timer opmodeTimer;
//    private PathChain fullAutoPath;
//    private boolean pathStarted = false;
//    private boolean Outtake;
//
//    @Override
//    public void init() {
//        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
//        follower = Constants.createFollower(hardwareMap);
//        follower.setStartingPose(new Pose(56, 8, Math.toRadians(90)));
//
//        robot = new AllMechs(hardwareMap, gamepad1, gamepad2);
//
//        opmodeTimer = new Timer();
//        opmodeTimer.resetTimer();
//
//        buildAutoPath();
//
//        panelsTelemetry.debug("Status", "Initialized");
//        panelsTelemetry.update(telemetry);
//    }
//
//    @Override
//    public void start() {
//        opmodeTimer.resetTimer();
//        pathStarted = false;
//    }
//
//    @Override
//    public void loop() {
//        // Start following the path on first loop iteration
//        if (!pathStarted) {
//            follower.followPath(fullAutoPath);
//            pathStarted = true;
//        }
//
//        follower.update();
//        CommandManager.INSTANCE.run();
//
//        panelsTelemetry.debug("X", follower.getPose().getX());
//        panelsTelemetry.debug("Y", follower.getPose().getY());
//        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
//        panelsTelemetry.debug("Is Busy", follower.isBusy());
//        panelsTelemetry.update(telemetry);
//    }
//
//    private void buildAutoPath() {
//        fullAutoPath = follower.pathBuilder()
//                // ========== PATH 1: Intaking 1 ==========
//
//                .addParametricCallback(1, ()->
//                        CommandManager.INSTANCE.scheduleCommand(
//                                new SequentialGroup(
//                                        robot.OuttakeOn(),
//                                        new Delay(2),
//                                        new SequentialGroup(
//                                                robot.transferOn(),
//                                                robot.doorOpen(),
//                                                robot.intakeOn()
//                                        ),
//                                        new Delay(5),
//                                        robot.ButtKicker(),
//                                        new ParallelGroup(
//                                             robot.OuttakeOff(),
//                                             robot.intakeOff(),
//                                             robot.transferOff(),
//                                             robot.doorClose()
//                                        )
//
//                                )
//                        ))
//                .addPath(new BezierLine(
//                        new Pose(56.000, 8.000),
////                        new Pose(37.685 + 24, 38.932),
//                        new Pose(57, 16) //new Pose(20.218 + 24, 35.438)
//                ))
//                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
//                // Start intake, transfer, and turret immediately at beginning
//                .addParametricCallback(0.0, () -> {
//                    CommandManager.INSTANCE.scheduleCommand(
//                            new ParallelGroup(
//                                    robot.intakeOn(),
//                                    robot.transferOn(),
//                                    robot.turretOn()
//                            )
//                    );
//                })
//                // Turn on outtake at 75% of path
//                .addParametricCallback(0.75, () -> {
//                    CommandManager.INSTANCE.scheduleCommand(robot.OuttakeOn());
//                })
//
//                // ========== PATH 2: Outtaking 1 (BALL 1) ==========
//                .addPath(new BezierCurve(
//                        new Pose(20.218 + 24, 35.438),
//                        new Pose(46.170 + 24, 43.674),
//                        new Pose(63.390 + 24, 73.872)
//                ))
//                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(145))
//                // Pause at end of path to score
//                .addParametricCallback(0.98, () -> {
//                    follower.pausePathFollowing();
//
//                    CommandManager.INSTANCE.scheduleCommand(
//                                    robot.doorOpen()
//                    );
//
//                    new Thread(() -> {
//                        try {
//
//                            Thread.sleep(500);
//                            CommandManager.INSTANCE.scheduleCommand(
//                                    robot.ButtKicker()
//                                    );
//                            Thread.sleep(200);
//                            CommandManager.INSTANCE.scheduleCommand(
//                                    new ParallelGroup(
//                                            robot.transferOff(),
//                                            robot.intakeOff(),
//                                            robot.ButtKicker(),
//                                            robot.OuttakeOff()
//                                    ));
//                            Thread.sleep(300);
//                            follower.resumePathFollowing();
//                        } catch (InterruptedException e) {
//                            Thread.currentThread().interrupt();
//                        }
//                    }).start();
//                })

//                // ========== PATH 3: Intaking 2 ==========
//                .addPath(new BezierCurve(
//                        new Pose(63.390 + 24, 73.872),
//                        new Pose(40.430 + 24, 48.915),
//                        new Pose(20.475 + 14, 62.388)
//                ))
//                .setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(180))
//                .addParametricCallback(0.0, () -> {
//                    CommandManager.INSTANCE.scheduleCommand(
//                            new ParallelGroup(
//                                    robot.intakeOn(),
//                                    robot.transferOn(),
//                                    robot.turretOn()
//                            )
//                    );
//                })
//                .addParametricCallback(0.75, () -> {
//                    CommandManager.INSTANCE.scheduleCommand(robot.OuttakeOn());
//                })
//
//                // ========== PATH 4: Outtaking 2 (BALL 2) ==========
//                .addPath(new BezierCurve(
//                        new Pose(20.475 + 14, 62.388),
//                        new Pose(45.421 + 24, 70.627),
//                        new Pose(51.161 + 24, 85.352)
//                ))
//                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(145))
//                .addParametricCallback(0.95, () -> {
//                    CommandManager.INSTANCE.scheduleCommand(
//                            new ParallelGroup(
//                                    robot.intakeOff(),
//                                    robot.transferOn(),
//                                    robot.doorOpen()
//                            )
//                    );
//                })
//                .addParametricCallback(0.98, () -> {
//                    CommandManager.INSTANCE.scheduleCommand(robot.buttkickerUp());
//                    new Thread(() -> {
//                        try {
//                            Thread.sleep(200);
//                            CommandManager.INSTANCE.scheduleCommand(robot.buttkickerDown());
//                        } catch (InterruptedException e) {
//                            Thread.currentThread().interrupt();
//                        }
//                    }).start();
//                })
//
//                // ========== PATH 5: Intaking 3 ==========
//                .addPath(new BezierCurve(
//                        new Pose(51.161 + 24, 85.352),
//                        new Pose(37.934 + 24, 81.608),
//                        new Pose(20.218 + 24, 81.355)
//                ))
//                .setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(180))
//                .addParametricCallback(0.0, () -> {
//                    CommandManager.INSTANCE.scheduleCommand(
//                            new ParallelGroup(
//                                    robot.intakeOn(),
//                                    robot.transferOn(),
//                                    robot.turretOn()
//                            )
//                    );
//                })
//                .addParametricCallback(0.75, () -> {
//                    CommandManager.INSTANCE.scheduleCommand(robot.OuttakeOn());
//                })
//
//                // ========== PATH 6: Outtaking 3 (BALL 3 - WITH SPECIAL HANDLING) ==========
//                .addPath(new BezierLine(
//                        new Pose(20.218 + 24, 81.355),
//                        new Pose(50.912 + 24, 85.352)
//                ))
//                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(145))
//                // For ball 3, wait until path is complete before starting outtake
//                .addPathEndTimeoutCallback(0.1, () -> {
//                    CommandManager.INSTANCE.scheduleCommand(
//                            new ParallelGroup(
//                                    robot.intakeOff(),
//                                    robot.transferOn(),
//                                    robot.doorOpen()
//                            )
//                    );
//                    // Delay the buttkicker activation
//                    new Thread(() -> {
//                        try {
//                            Thread.sleep(500); // Wait 500ms before kicking
//                            CommandManager.INSTANCE.scheduleCommand(robot.buttkickerUp());
//                            Thread.sleep(200);
//                            CommandManager.INSTANCE.scheduleCommand(robot.buttkickerDown());
//                        } catch (InterruptedException e) {
//                            Thread.currentThread().interrupt();
//                        }
//                    }).start();
//                })
//
//                // ========== PATH 7: Intaking 4 ==========
//                .addPath(new BezierCurve(
//                        new Pose(50.912 + 24, 85.352),
//                        new Pose(9.983 + 24, 41.428),
//                        new Pose(7.737 + 24, 8.485)
//                ))
//                .setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(270))
//                .addParametricCallback(0.0, () -> {
//                    CommandManager.INSTANCE.scheduleCommand(
//                            new ParallelGroup(
//                                    robot.intakeOn(),
//                                    robot.transferOn(),
//                                    robot.turretOn()
//                            )
//                    );
//                })
//                .addParametricCallback(0.75, () -> {
//                    CommandManager.INSTANCE.scheduleCommand(robot.OuttakeOn());
//                })
//
//                // ========== PATH 8: Outtaking 4 (BALL 4) ==========
//                .addPath(new BezierCurve(
//                        new Pose(7.737 + 24, 8.485),
//                        new Pose(41.428 + 24, 14.974),
//                        new Pose(55.154 + 24, 11.730)
//                ))
//                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(90))
//                .addParametricCallback(0.95, () -> {
//                    CommandManager.INSTANCE.scheduleCommand(
//                            new ParallelGroup(
//                                    robot.intakeOff(),
//                                    robot.transferOn(),
//                                    robot.doorOpen()
//                            )
//                    );
//                })
//                .addParametricCallback(0.98, () -> {
//                    CommandManager.INSTANCE.scheduleCommand(robot.buttkickerUp());
//                    new Thread(() -> {
//                        try {
//                            Thread.sleep(200);
//                            CommandManager.INSTANCE.scheduleCommand(robot.buttkickerDown());
//                        } catch (InterruptedException e) {
//                            Thread.currentThread().interrupt();
//                        }
//                    }).start();
//                })
//
//                // ========== PATH 9: Leave ==========
//                .addPath(new BezierLine(
//                        new Pose(55.154 + 24, 11.730),
//                        new Pose(55.404 + 24, 28.201)
//                ))
//                .setConstantHeadingInterpolation(Math.toRadians(90))
//                .build();
//    }
//}






//package org.firstinspires.ftc.teamcode.paths;
//
//import com.bylazar.configurables.annotations.Configurable;
//import com.bylazar.telemetry.PanelsTelemetry;
//import com.bylazar.telemetry.TelemetryManager;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.BezierCurve;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.PathChain;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//
//@Autonomous(name = "Red15Close", group = "Autonomous")
//@Configurable // Panels
//public class Red15CloseAuto extends OpMode {
//
//    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
//    public Follower follower; // Pedro Pathing follower instance
//    private int pathState; // Current autonomous path state (state machine)
//    private Paths paths; // Paths defined in the Paths class
//
//    @Override
//    public void init() {
//        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
//
//        follower = Constants.createFollower(hardwareMap);
//        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));
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
//        pathState = autonomousPathUpdate(); // Update autonomous state machine
//
//        // Log values to Panels and Driver Station
//        panelsTelemetry.debug("Path State", pathState);
//        panelsTelemetry.debug("X", follower.getPose().getX());
//        panelsTelemetry.debug("Y", follower.getPose().getY());
//        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
//        panelsTelemetry.update(telemetry);
//    }
//
//    public static class Paths {
//
//        public PathChain ShootingPreload;
//        public PathChain Intaking1;
//        public PathChain Outtaking1;
//        public PathChain Intaking2;
//        public PathChain Outtaking2;
//        public PathChain Intaking3;
//        public PathChain Outtaking3;
//        public PathChain Intaking4;
//        public PathChain Outtaking4;
//        public PathChain Leave;
//
//        public Paths(Follower follower) {
//            ShootingPreload = follower
//                    .pathBuilder()
//                    .addPath(
//                            new BezierLine(new Pose(127.823, 118.807), new Pose(89.370, 82.475))
//                    )
//                    .setConstantHeadingInterpolation(Math.toRadians(45))
//                    .build();
//
//            Intaking1 = follower
//                    .pathBuilder()
//                    .addPath(
//                            new BezierCurve(
//                                    new Pose(89.370, 82.475),
//                                    new Pose(103.079, 80.956),
//                                    new Pose(125.862, 83.389)
//                            )
//                    )
//                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
//                    .build();
//
//            Outtaking1 = follower
//                    .pathBuilder()
//                    .addPath(
//                            new BezierLine(new Pose(125.862, 83.389), new Pose(90.431, 82.740))
//                    )
//                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
//                    .build();
//
//            Intaking2 = follower
//                    .pathBuilder()
//                    .addPath(
//                            new BezierCurve(
//                                    new Pose(90.431, 82.740),
//                                    new Pose(97.770, 44.016),
//                                    new Pose(130.286, 64.587)
//                            )
//                    )
//                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
//                    .build();
//
//            Outtaking2 = follower
//                    .pathBuilder()
//                    .addPath(
//                            new BezierCurve(
//                                    new Pose(130.286, 64.587),
//                                    new Pose(101.039, 62.586),
//                                    new Pose(89.901, 83.536)
//                            )
//                    )
//                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
//                    .build();
//
//            Intaking3 = follower
//                    .pathBuilder()
//                    .addPath(
//                            new BezierCurve(
//                                    new Pose(89.901, 83.536),
//                                    new Pose(80.354, 28.110),
//                                    new Pose(126.762, 34.740)
//                            )
//                    )
//                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
//                    .build();
//
//            Outtaking3 = follower
//                    .pathBuilder()
//                    .addPath(
//                            new BezierCurve(
//                                    new Pose(126.762, 34.740),
//                                    new Pose(98.652, 46.144),
//                                    new Pose(89.901, 82.740)
//                            )
//                    )
//                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
//                    .build();
//
//            Intaking4 = follower
//                    .pathBuilder()
//                    .addPath(
//                            new BezierCurve(
//                                    new Pose(89.901, 82.740),
//                                    new Pose(128.884, 48.000),
//                                    new Pose(136.480, 8.624)
//                            )
//                    )
//                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(270))
//                    .build();
//
//            Outtaking4 = follower
//                    .pathBuilder()
//                    .addPath(
//                            new BezierCurve(
//                                    new Pose(136.480, 8.624),
//                                    new Pose(114.360, 16.145),
//                                    new Pose(86.047, 12.606)
//                            )
//                    )
//                    .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(60))
//                    .build();
//
//            Leave = follower
//                    .pathBuilder()
//                    .addPath(
//                            new BezierLine(new Pose(86.047, 12.606), new Pose(86.453, 32.884))
//                    )
//                    .setLinearHeadingInterpolation(Math.toRadians(60), Math.toRadians(90))
//                    .build();
//        }
//    }
//
//    public int autonomousPathUpdate() {
//        switch (pathState) {
//            case 0:
//                follower.followPath(paths.ShootingPreload);
//                pathState = 1;
//                break;
//            case 1:
//                if(!follower.isBusy()) {
//                    /* Grab Sample 1 */
//                    follower.followPath(paths.Intaking1, true);
//
//                }
//                pathState = 2;
//                break;
//            case 2:
//                if(!follower.isBusy()) {
//                    /* Grab Sample 1 */
//                    follower.followPath(paths.Outtaking1, true);
//                    pathState = 3;
//                }
//                break;
//            case 3:
//                if(!follower.isBusy()) {
//                    /* Score Sample 1 */
//                    follower.followPath(paths.Intaking2, true);
//                    pathState = 4;
//                }
//                break;
//            case 4:
//                if(!follower.isBusy()) {
//                    /* Grab Sample 2 */
//                    follower.followPath(paths.Outtaking2, true);
//                    pathState = 5;
//                }
//                break;
//            case 5:
//                if(!follower.isBusy()) {
//                    /* Score Sample 2 */
//                    follower.followPath(paths.Intaking3, true);
//                    pathState = 6;
//                }
//                break;
//            case 6:
//                if(!follower.isBusy()) {
//                    /* Grab Sample 3 */
//                    follower.followPath(paths.Outtaking3, true);
//                    pathState = 7;
//                }
//                break;
//            case 7:
//                if(!follower.isBusy()) {
//                    /* Score Sample 3 */
//                    follower.followPath(paths.Intaking4, true);
//                    pathState = 8;
//                }
//                break;
//            case 8:
//                if(!follower.isBusy()) {
//                    /* Grab Sample 4 */
//                    follower.followPath(paths.Outtaking4, true);
//                    pathState = 9;
//                }
//                break;
//            case 9:
//                if(!follower.isBusy()) {
//                    /* Score Sample 4 */
//                    follower.followPath(paths.Leave, true);
//                    pathState = 10;
//                }
//                break;
//            case 10:
//                if(!follower.isBusy()) {
//                    /* Parked - End autonomous */
//                    pathState = -1;
//                }
//                break;
//        }
//        return pathState;
//    }
//}
//







//package org.firstinspires.ftc.teamcode.paths;
//
//import com.bylazar.configurables.annotations.Configurable;
//import com.bylazar.telemetry.PanelsTelemetry;
//import com.bylazar.telemetry.TelemetryManager;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.BezierCurve;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.PathChain;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//
//@Autonomous(name = "Red15Far", group = "Autonomous")
//@Configurable // Panels
//public class Red15FarAuto extends OpMode {
//
//    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
//    public Follower follower; // Pedro Pathing follower instance
//    private int pathState; // Current autonomous path state (state machine)
//    private Paths paths; // Paths defined in the Paths class
//
//    @Override
//    public void init() {
//        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
//
//        follower = Constants.createFollower(hardwareMap);
//        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));
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
//        pathState = autonomousPathUpdate(); // Update autonomous state machine
//
//        // Log values to Panels and Driver Station
//        panelsTelemetry.debug("Path State", pathState);
//        panelsTelemetry.debug("X", follower.getPose().getX());
//        panelsTelemetry.debug("Y", follower.getPose().getY());
//        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
//        panelsTelemetry.update(telemetry);
//    }
//
//    public static class Paths {
//
//        public PathChain Intaking1;
//        public PathChain Outtaking1;
//        public PathChain Intaking2;
//        public PathChain Outtaking2;
//        public PathChain Intaking3;
//        public PathChain Outtaking3;
//        public PathChain Intaking4;
//        public PathChain Outtaking4;
//        public PathChain Leave;
//
//        public Paths(Follower follower) {
//            Intaking1 = follower
//                    .pathBuilder()
//                    .addPath(
//                            new BezierCurve(
//                                    new Pose(88.000, 8.000),
//                                    new Pose(100.203, 36.053),
//                                    new Pose(125.782, 35.438)
//                            )
//                    )
//                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
//                    .build();
//
//            Outtaking1 = follower
//                    .pathBuilder()
//                    .addPath(
//                            new BezierCurve(
//                                    new Pose(125.782, 35.438),
//                                    new Pose(90.961, 53.039),
//                                    new Pose(80.610, 73.872)
//                            )
//                    )
//                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
//                    .build();
//
//            Intaking2 = follower
//                    .pathBuilder()
//                    .addPath(
//                            new BezierCurve(
//                                    new Pose(80.610, 73.872),
//                                    new Pose(109.715, 45.785),
//                                    new Pose(129.844, 64.366)
//                            )
//                    )
//                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
//                    .build();
//
//            Outtaking2 = follower
//                    .pathBuilder()
//                    .addPath(
//                            new BezierCurve(
//                                    new Pose(129.844, 64.366),
//                                    new Pose(99.318, 67.463),
//                                    new Pose(92.839, 85.352)
//                            )
//                    )
//                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(25))
//                    .build();
//
//            Intaking3 = follower
//                    .pathBuilder()
//                    .addPath(
//                            new BezierCurve(
//                                    new Pose(92.839, 85.352),
//                                    new Pose(106.066, 81.608),
//                                    new Pose(125.782, 83.355)
//                            )
//                    )
//                    .setLinearHeadingInterpolation(Math.toRadians(25), Math.toRadians(0))
//                    .build();
//
//            Outtaking3 = follower
//                    .pathBuilder()
//                    .addPath(
//                            new BezierLine(new Pose(125.782, 83.355), new Pose(92.818, 84.862))
//                    )
//                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
//                    .build();
//
//            Intaking4 = follower
//                    .pathBuilder()
//                    .addPath(
//                            new BezierCurve(
//                                    new Pose(92.818, 84.862),
//                                    new Pose(136.922, 55.961),
//                                    new Pose(136.037, 9.288)
//                            )
//                    )
//                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(90))
//                    .build();
//
//            Outtaking4 = follower
//                    .pathBuilder()
//                    .addPath(
//                            new BezierCurve(
//                                    new Pose(136.037, 9.288),
//                                    new Pose(118.276, 13.525),
//                                    new Pose(88.037, 11.942)
//                            )
//                    )
//                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(60))
//                    .build();
//
//            Leave = follower
//                    .pathBuilder()
//                    .addPath(
//                            new BezierLine(new Pose(88.037, 11.942), new Pose(88.575, 34.475))
//                    )
//                    .setLinearHeadingInterpolation(Math.toRadians(60), Math.toRadians(90))
//                    .build();
//        }
//    }
//
//    public int autonomousPathUpdate() {
//        switch (pathState) {
//            case 0:
//                follower.followPath(paths.Intaking1);
//                pathState = 1;
//                break;
//            case 1:
//                if(!follower.isBusy()) {
//                    /* Grab Sample 1 */
//                    follower.followPath(paths.Outtaking1, true);
//                    pathState = 2;
//                }
//                break;
//            case 2:
//                if(!follower.isBusy()) {
//                    /* Score Sample 1 */
//                    follower.followPath(paths.Intaking2, true);
//                    pathState = 3;
//                }
//                break;
//            case 3:
//                if(!follower.isBusy()) {
//                    /* Grab Sample 2 */
//                    follower.followPath(paths.Outtaking2, true);
//                    pathState = 4;
//                }
//                break;
//            case 4:
//                if(!follower.isBusy()) {
//                    /* Score Sample 2 */
//                    follower.followPath(paths.Intaking3, true);
//                    pathState = 5;
//                }
//                break;
//            case 5:
//                if(!follower.isBusy()) {
//                    /* Grab Sample 3 */
//                    follower.followPath(paths.Outtaking3, true);
//                    pathState = 6;
//                }
//                break;
//            case 6:
//                if(!follower.isBusy()) {
//                    /* Score Sample 3 */
//                    follower.followPath(paths.Intaking4, true);
//                    pathState = 7;
//                }
//                break;
//            case 7:
//                if(!follower.isBusy()) {
//                    /* Grab Sample 4 */
//                    follower.followPath(paths.Outtaking4, true);
//                    pathState = 8;
//                }
//                break;
//            case 8:
//                if(!follower.isBusy()) {
//                    /* Score Sample 4 */
//                    follower.followPath(paths.Leave, true);
//                    pathState = 9;
//                }
//                break;
//            case 9:
//                if(!follower.isBusy()) {
//                    /* Parked - End autonomous */
//                    pathState = -1;
//                }
//                break;
//        }
//        return pathState;
//    }
//}
//
