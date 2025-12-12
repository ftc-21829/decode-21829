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

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.AllMechs;

@Autonomous(name = "Blue15Far", group = "Autonomous")
@Configurable // Panels
public class Blue15FarAuto extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private int actionState;
    private AllMechs robot;
    private boolean actionStarted = false;
    private boolean pathStarted = false;

    private boolean actionComplete = false;

    private Timer pathTimer, actionTimer, opmodeTimer;

    private Paths paths; // Paths defined in the Paths class

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));
        pathState = 0;

        robot = new AllMechs(hardwareMap, gamepad1, gamepad2);

        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        autonomousPathUpdate(); // Update autonomous state machine

        CommandManager.INSTANCE.run();

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {

        public PathChain Intaking1;
        public PathChain Outtaking1;
        public PathChain Intaking2;
        public PathChain GatePose;
        public PathChain Outtaking2;
        public PathChain Intaking3;
        public PathChain Outtaking3;
        public PathChain Intaking4;
        public PathChain Outtake4;
        public PathChain Leave;

        public Paths(Follower follower) {
            Intaking1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(56.000, 8.000),
                                    new Pose(45.088, 36.352),
                                    new Pose(18.218, 35.507)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                    .build();

            Outtaking1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(18.218, 35.507),
                                    new Pose(55.515, 52.415),
                                    new Pose(59.742, 76.368)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(132))
                    .build();

            Intaking2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(59.742, 76.368),
                                    new Pose(55.796, 55.796),
                                    new Pose(18.218, 59.74) //67.388
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(132), Math.toRadians(180))
                    .build();

            GatePose = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(18.218, 59.74),
                                    new Pose(14.935, 64.532) //67.388
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            Outtaking2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(14.935, 64.532),
                                    new Pose(41.988, 66.787),
                                    new Pose(51.851, 84.258)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(145))
                    .build();

            Intaking3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(51.851, 84.258),
                                    new Pose(41.706, 82.578),
                                    new Pose(18.218, 83.413)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(180))
                    .build();

            Outtaking3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(18.218, 83.413),
                                    new Pose(50.442, 88.767)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(145))
                    .build();

            Intaking4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(50.442, 88.767),
                                    new Pose(20.008, 72.705),
                                    new Pose(9.299, 9.581)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(270))
                    .build();

            Outtake4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(9.299, 9.581),
                                    new Pose(38.607, 21.699),
                                    new Pose(57.769, 15.217)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(120))
                    .build();

            Leave = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(57.769, 15.217),
                                    new Pose(41.723, 62.315),
                                    new Pose(22.691, 70.109)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(90))
                    .build();
        }
    }
    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(paths.Intaking1);
                pathState = 1;
                break;
            case 1:
                if(!follower.isBusy()) {
                    /* Grab Sample 1 */
                    follower.followPath(paths.Outtaking1, true);

                }
                pathState = 2;
                break;
            case 2:
                if(!follower.isBusy()) {
                    /* Grab Sample 1 */
                    follower.followPath(paths.Intaking2, true);
                    pathState = 3;
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    /* Score Sample 1 */
                    follower.followPath(paths.GatePose, true);
                    pathState = 4;
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    /* Grab Sample 2 */
                    follower.followPath(paths.Outtaking2, true);
                    pathState = 5;
                }
                break;
            case 5:
                if(!follower.isBusy()) {
                    /* Score Sample 2 */
                    follower.followPath(paths.Intaking3, true);
                    pathState = 6;
                }
                break;
            case 6:
                if(!follower.isBusy()) {
                    /* Grab Sample 3 */
                    follower.followPath(paths.Outtaking3, true);
                    pathState = 7;
                }
                break;
            case 7:
                if(!follower.isBusy()) {
                    /* Score Sample 3 */
                    follower.followPath(paths.Intaking4, true);
                    pathState = 8;
                }
                break;
            case 8:
                if(!follower.isBusy()) {
                    /* Grab Sample 4 */
                    follower.followPath(paths.Outtake4, true);
                    pathState = 9;
                }
                break;
            case 9:
                if(!follower.isBusy()) {
                    /* Score Sample 4 */
                    follower.followPath(paths.Leave, true);
                    pathState = 10;
                }
                break;
            case 10:
                if(!follower.isBusy()) {
                    /* Parked - End autonomous */
                    pathState = -1;
                }
                break;
        }
        return pathState;
    }

//    public void autonomousPathUpdate() {
//
//        switch (pathState) {
//
//            case 0:
//                if (!pathStarted) {
//                    follower.followPath(paths.Intaking1);
//                    actionComplete = false;
//                    actionStarted = false;
//                    pathStarted = true;
//                }
//
//                // Move on only AFTER both are done
//                if (!follower.isBusy() && actionComplete) {
//                    pathState = 1;
//                    pathStarted = false;
//                }
//                break;
//
//            case 1:
//                if (!pathStarted) {
//                    follower.followPath(paths.Outtaking1, true);
//                    actionComplete = false;
//                    actionStarted = false;
//                    pathStarted = true;
//                }
//
//                if (!follower.isBusy() && actionComplete) {
//                    pathState = 2;
//                    pathStarted = false;
//                }
//                break;
//
//            case 2:
//                if (!pathStarted) {
//                    follower.followPath(paths.Intaking2, true);
//                    actionComplete = false;
//                    actionStarted = false;
//                    pathStarted = true;
//                }
//
//                if (!follower.isBusy() && actionComplete) {
//                    pathState = 3;
//                    pathStarted = false;
//                }
//                break;
//
//            case 3:
//                if (!pathStarted) {
//                    follower.followPath(paths.GatePose, true);
//                    actionComplete = false;
//                    actionStarted = false;
//                    pathStarted = true;
//                }
//
//                if (!follower.isBusy() && actionComplete) {
//                    pathState = 3;
//                    pathStarted = false;
//                }
//                break;
//
//            case 4:
//                if (!pathStarted) {
//                    follower.followPath(paths.Outtaking2, true);
//                    actionComplete = false;
//                    actionStarted = false;
//                    pathStarted = true;
//                }
//
//                if (!follower.isBusy() && actionComplete) {
//                    pathState = 4;
//                    pathStarted = false;
//                }
//                break;
//
//            case 5:
//                if (!pathStarted) {
//                    follower.followPath(paths.Intaking3, true);
//                    actionComplete = false;
//                    actionStarted = false;
//                    pathStarted = true;
//                }
//
//                if (!follower.isBusy() && actionComplete) {
//                    pathState = 5;
//                    pathStarted = false;
//                }
//                break;
//
//            case 6:
//                if (!pathStarted) {
//                    follower.followPath(paths.Outtaking3, true);
//                    actionComplete = false;
//                    actionStarted = false;
//                    pathStarted = true;
//                }
//
//                if (!follower.isBusy() && actionComplete) {
//                    pathState = 6;
//                    pathStarted = false;
//                }
//                break;
//
//            case 7:
//                if (!pathStarted) {
//                    follower.followPath(paths.Intaking4, true);
//                    actionComplete = false;
//                    actionStarted = false;
//                    pathStarted = true;
//                }
//
//                if (!follower.isBusy() && actionComplete) {
//                    pathState = 7;
//                    pathStarted = false;
//                }
//                break;
//
//            case 8:
//                if (!pathStarted) {
//                    follower.followPath(paths.Outtake4, true);
//                    actionComplete = false;
//                    actionStarted = false;
//                    pathStarted = true;
//                }
//
//                if (!follower.isBusy() && actionComplete) {
//                    pathState = 8;
//                    pathStarted = false;
//                }
//                break;
//
//            case 9:
//                if (!pathStarted) {
//                    follower.followPath(paths.Leave, true);
//                    actionComplete = false;
//                    actionStarted = false;
//                    pathStarted = true;
//                }
//
//                if (!follower.isBusy()) {
//                    pathState = 9;
//                }
//                break;
//
//            case 10:
//                // done
//                break;
//        }
//    }


//    public void autonomousActionUpdate() {
//
//        switch (pathState) {
//
//            // ---------- INTAKING 1 ----------
//            case 0:
//                if (!actionStarted) {
//                    actionStarted = true;
//                    actionTimer.resetTimer();
//
//                    CommandManager.INSTANCE.scheduleCommand(
//                            new ParallelGroup(
//                                    robot.intakeOn(),
//                                    robot.transferOn()
//                            )
//                    );
//                }
//
//                if (actionStarted && actionTimer.getElapsedTimeSeconds() > 0.75) {
//
//                    CommandManager.INSTANCE.scheduleCommand(
//                            new ParallelGroup(
//                                    robot.intakeOff(),
//                                    robot.transferOff(),
//                                    robot.OuttakeOn()
//                            )
//                    );
//
//                    actionComplete = true;
//                }
//                break;
//
//            // ---------- OUTTAKING 1 ----------
//            case 1:
//                if (!actionStarted) {
//                    actionStarted = true;
//                    actionTimer.resetTimer();
//
//                    CommandManager.INSTANCE.scheduleCommand(
//                            new ParallelGroup(
//                                    robot.doorOpen(),
//                                    robot.transferOn()
//                            )
//                    );
//                }
//
//                if (actionStarted && actionTimer.getElapsedTimeSeconds() > 1.5) {
//                    CommandManager.INSTANCE.scheduleCommand(robot.ButtKicker());
//                    actionComplete = true;
//                }
//                break;
//
//            // ---------- INTAKING 2 ----------
//            case 2:
//                if (!actionStarted) {
//                    actionStarted = true;
//                    actionTimer.resetTimer();
//
//                    CommandManager.INSTANCE.scheduleCommand(
//                            new ParallelGroup(
//                                    robot.intakeOn(),
//                                    robot.transferOn()
//                            )
//                    );
//                }
//
//                if (actionStarted && actionTimer.getElapsedTimeSeconds() > 0.75) {
//                    CommandManager.INSTANCE.scheduleCommand(
//                            new ParallelGroup(
//                                    robot.intakeOff(),
//                                    robot.transferOff(),
//                                    robot.OuttakeOn()
//                            )
//                    );
//                    actionComplete = true;
//                }
//                break;
//
//            // ---------- OUTTAKING 2 ----------
//            case 3:
//                if (!actionStarted) {
//                    actionStarted = true;
//                    actionTimer.resetTimer();
//
//                    CommandManager.INSTANCE.scheduleCommand(
//                            new ParallelGroup(
//                                    robot.doorOpen(),
//                                    robot.transferOn()
//                            )
//                    );
//                }
//
//                if (actionStarted && actionTimer.getElapsedTimeSeconds() > 1.5) {
//                    CommandManager.INSTANCE.scheduleCommand(robot.ButtKicker());
//                    actionComplete = true;
//                }
//                break;
//
//            // ---------- INTAKING 3 ----------
//            case 4:
//                if (!actionStarted) {
//                    actionStarted = true;
//                    actionTimer.resetTimer();
//
//                    CommandManager.INSTANCE.scheduleCommand(
//                            new ParallelGroup(
//                                    robot.intakeOn(),
//                                    robot.transferOn()
//                            )
//                    );
//                }
//
//                if (actionStarted && actionTimer.getElapsedTimeSeconds() > 0.75) {
//                    CommandManager.INSTANCE.scheduleCommand(
//                            new ParallelGroup(
//                                    robot.intakeOff(),
//                                    robot.transferOff(),
//                                    robot.OuttakeOn()
//                            )
//                    );
//                    actionComplete = true;
//                }
//                break;
//
//            // ---------- OUTTAKING 3 ----------
//            case 5:
//                if (!actionStarted) {
//                    actionStarted = true;
//                    actionTimer.resetTimer();
//
//                    CommandManager.INSTANCE.scheduleCommand(
//                            new ParallelGroup(
//                                    robot.doorOpen(),
//                                    robot.transferOn()
//                            )
//                    );
//                }
//
//                if (actionStarted && actionTimer.getElapsedTimeSeconds() > 1.5) {
//                    CommandManager.INSTANCE.scheduleCommand(robot.ButtKicker());
//
//                    actionComplete = true;
//                }
//                break;
//
//            // ---------- INTAKING 4 ----------
//            case 6:
//                if (!actionStarted) {
//                    actionStarted = true;
//                    actionTimer.resetTimer();
//
//                    CommandManager.INSTANCE.scheduleCommand(
//                            new ParallelGroup(
//                                    robot.intakeOn(),
//                                    robot.transferOn()
//                            )
//                    );
//                }
//
//                if (actionStarted && actionTimer.getElapsedTimeSeconds() > 0.75) {
//                    CommandManager.INSTANCE.scheduleCommand(
//                            new ParallelGroup(
//                                    robot.intakeOff(),
//                                    robot.transferOff(),
//                                    robot.OuttakeOn()
//                            )
//                    );
//                    actionComplete = true;
//                }
//                break;
//
//            // ---------- OUTTAKING 4 ----------
//            case 7:
//                if (!actionStarted) {
//                    actionStarted = true;
//                    actionTimer.resetTimer();
//
//                    CommandManager.INSTANCE.scheduleCommand(
//                            new ParallelGroup(
//                                    robot.doorOpen(),
//                                    robot.transferOn()
//                            )
//                    );
//                }
//
//                if (actionStarted && actionTimer.getElapsedTimeSeconds() > 1.5) {
//                    CommandManager.INSTANCE.scheduleCommand(robot.ButtKicker());
//
//                    actionComplete = true;
//                }
//                break;
//        }
//    }



//    public void setActionState(int aState) {
//        actionState = aState;
//        actionTimer.resetTimer();
//    }
//    @Override
//    public void start() {
//        opmodeTimer.resetTimer();
//    }
}
