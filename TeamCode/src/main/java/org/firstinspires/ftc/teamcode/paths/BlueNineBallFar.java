package org.firstinspires.ftc.teamcode.paths;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.rowanmcalpin.nextftc.core.command.CommandManager;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.AllMechs;


@Autonomous(name = "FourBlueFar", group = "Blue_side")
public class BlueNineBallFar  extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;


    private int pathState;
    //see if push in is needed
    private final Pose startPose = new Pose(56, 8, Math.toRadians(90));
    private final Pose score1Pose = new Pose(58.9, 13.48, Math.toRadians(115));
    private final Pose pickup1Pose = new Pose(14.724, 35.688, Math.toRadians(180));
    private final Pose score2Pose = new Pose(58.9, 13.48, Math.toRadians(115));
    private final Pose pickup2Pose = new Pose(8.98, 8.485, Math.toRadians(270));
    private final Pose score3Pose = new Pose(59.15, 12.977, Math.toRadians(115));
    private final Pose parkPose = new Pose(55.903, 29.948, Math.toRadians(90));



    private Path scorePreload;
    public PathChain pickup1, score2, pickup2, score3, park;

    public void buildPaths() {

        scorePreload = new Path(new BezierLine(startPose, score1Pose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), score1Pose.getHeading());


        pickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(score1Pose, pickup1Pose, new Pose(44.423, 39.4315)))
                .setLinearHeadingInterpolation(score1Pose.getHeading(), pickup1Pose.getHeading())
                .setBrakingStart(4)
                .setBrakingStrength(0.04)
                .build();

        score2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, score2Pose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), score2Pose.getHeading())
                .setBrakingStart(4)
                .setBrakingStrength(0.04)
                .build();

        pickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(score2Pose, pickup2Pose, new Pose(9.7331, 35.938)))
                .setLinearHeadingInterpolation(score2Pose.getHeading(), pickup2Pose.getHeading())
                .setBrakingStart(4)
                .setBrakingStrength(0.04)
                .build();

        score3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, score3Pose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), score3Pose.getHeading())
                .setBrakingStart(4)
                .setBrakingStrength(0.04)
                .build();


        park = follower.pathBuilder()
                .addPath(new BezierLine(score3Pose, parkPose))
                .setLinearHeadingInterpolation(score3Pose.getHeading(), parkPose.getHeading())
                .setBrakingStart(4)
                .setBrakingStrength(0.04)
                .build();

    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:
            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Preload */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(pickup1,true);
                    setPathState(2);
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(score2,true);
                    setPathState(3);
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(pickup2,true);
                    setPathState(4);
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(score3,true);
                    setPathState(5);
                }
                break;
            case 5:

                if(!follower.isBusy()) {
                    follower.followPath(park, true);
                }
                break;
            case 8:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setPathState(-1);
                }
                break;
        }
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }


    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();


    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        setPathState(0);

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();



    }

    public void start() {
        opmodeTimer.resetTimer();
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
//import com.rowanmcalpin.nextftc.core.command.utility.InstantCommand;
//
//import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//import org.firstinspires.ftc.teamcode.subsystems.AllMechs;
//
//@Autonomous(name = "Blue15Far", group = "Autonomous")
//@Configurable // Panels
//public class Blue15FarAuto extends OpMode {
//    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
//    public Follower follower; // Pedro Pathing follower instance
//    private int pathState; // Current autonomous path state (state machine)
//    private int actionState;
//    private AllMechs robot;
//
//    private Timer pathTimer, actionTimer, opmodeTimer;
//
//    private Paths paths; // Paths defined in the Paths class
//
//    @Override
//    public void init() {
//        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
//        follower = Constants.createFollower(hardwareMap);
//        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));
//
//        robot = new AllMechs(hardwareMap, gamepad1, gamepad2);
//
//        pathTimer = new Timer();
//        actionTimer = new Timer();
//        opmodeTimer = new Timer();
//        opmodeTimer.resetTimer();
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
//        autonomousActionUpdate();
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
//        public PathChain Outtake4;
//        public PathChain Leave;
//
//        public Paths(Follower follower) {
//            Intaking1 = follower
//                    .pathBuilder()
//                    .addPath(
//                            new BezierCurve(
//                                    new Pose(56.000+24, 8.000),
//                                    new Pose(37.685+24, 38.932),
//                                    new Pose(20.218+24, 35.438)
//                            )
//                    )
//                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
//                    .build();
//
//            Outtaking1 = follower
//                    .pathBuilder()
//                    .addPath(
//                            new BezierCurve(
//                                    new Pose(20.218+24, 35.438),
//                                    new Pose(46.170+24, 43.674),
//                                    new Pose(63.390+24, 73.872)
//                            )
//                    )
//                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(145))
//                    .build();
//
//            Intaking2 = follower
//                    .pathBuilder()
//                    .addPath(
//                            new BezierCurve(
//                                    new Pose(63.390+24, 73.872),
//                                    new Pose(40.430+24, 48.915),
//                                    new Pose(20.475+14, 62.388) //67.388
//                            )
//                    )
//                    .setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(180))
//                    .build();
//
//            Outtaking2 = follower
//                    .pathBuilder()
//                    .addPath(
//                            new BezierCurve(
//                                    new Pose(20.475+14, 62.388),
//                                    new Pose(45.421+24, 70.627),
//                                    new Pose(51.161+24, 85.352)
//                            )
//                    )
//                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(145))
//                    .build();
//
//            Intaking3 = follower
//                    .pathBuilder()
//                    .addPath(
//                            new BezierCurve(
//                                    new Pose(51.161+24, 85.352),
//                                    new Pose(37.934+24, 81.608),
//                                    new Pose(20.218+24, 81.355)
//                            )
//                    )
//                    .setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(180))
//                    .build();
//
//            Outtaking3 = follower
//                    .pathBuilder()
//                    .addPath(
//                            new BezierLine(new Pose(20.218+24, 81.355), new Pose(50.912+24, 85.352))
//                    )
//                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(145))
//                    .build();
//
//            Intaking4 = follower
//                    .pathBuilder()
//                    .addPath(
//                            new BezierCurve(
//                                    new Pose(50.912+24, 85.352),
//                                    new Pose(9.983+24, 41.428),
//                                    new Pose(7.737+24, 8.485)
//                            )
//                    )
//                    .setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(270))
//                    .build();
//
//            Outtake4 = follower
//                    .pathBuilder()
//                    .addPath(
//                            new BezierCurve(
//                                    new Pose(7.737+24, 8.485),
//                                    new Pose(41.428+24, 14.974),
//                                    new Pose(55.154+24, 11.730)
//                            )
//                    )
//                    .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(90))
//                    .build();
//
//            Leave = follower
//                    .pathBuilder()
//                    .addPath(
//                            new BezierLine(new Pose(55.154+24, 11.730), new Pose(55.404+24, 28.201))
//                    )
//                    .setConstantHeadingInterpolation(Math.toRadians(90))
//                    .build();
//        }
//    }
//
//    public void autonomousPathUpdate() {
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
//                    follower.followPath(paths.Outtake4, true);
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
//    }
//
//    public void autonomousActionUpdate() {
//        switch (actionState) {
//
//            case 0:
//                if (actionTimer.getElapsedTimeSeconds() > 0) {
//                    CommandManager.INSTANCE.scheduleCommand(
//                            new ParallelGroup(
//                                    robot.intakeOn(),
//                                    robot.transferOn()
//                            )
//                    );
//                    setActionState(1);
//                }
//                break;
//            case 1:
//                // wait 0.5 seconds before dropping pixel
//                if (actionTimer.getElapsedTimeSeconds() > 0.75) {
//                    CommandManager.INSTANCE.scheduleCommand(
//                            new ParallelGroup(
//                                    robot.intakeOn(),
//                                    robot.transferOn()
//                            )
//                    );
//                    setActionState(2);
//                }
//
//            case 2:
//                // retract outtake after scoring
//                if (actionTimer.getElapsedTimeSeconds() > 0.3) {
//                    // robot.retract();
//                    setActionState(-1);  // done
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
//}


