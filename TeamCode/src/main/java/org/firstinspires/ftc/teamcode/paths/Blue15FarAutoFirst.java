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

@Autonomous(name = "Blue15Fartestthing", group = "Autonomous")
@Configurable
public class Blue15FarAutoFirst extends OpMode {
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
        follower.setStartingPose(new Pose(56, 8, Math.toRadians(90)));
        pathState = 0;


        robot = new AllMechs(hardwareMap, gamepad1, gamepad2);
        actionTimer = new Timer();

        // Build the leave path
        leavePath = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(56, 8, Math.toRadians(90)),
                                new Pose(56, 12, Math.toRadians(90))
                        )
                )
//                .setConstantHeadingInterpolation(Math.toRadians(90))
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

                }
                break;



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