//package org.firstinspires.ftc.teamcode.paths;
//
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.BezierCurve;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.Path;
//import com.pedropathing.paths.PathChain;
//import com.pedropathing.util.Timer;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//
//import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//
//
//@Autonomous(name = "FourBlueFar", group = "Blue_side")
//public class BlueNineBallFar  extends OpMode {
//
//    private Follower follower;
//    private Timer pathTimer, actionTimer, opmodeTimer;
//
//
//    private int pathState;
//    //see if push in is needed
//    private final Pose startPose = new Pose(56, 8, Math.toRadians(90));
//    private final Pose score1Pose = new Pose(58.9, 13.48, Math.toRadians(115));
//    private final Pose pickup1Pose = new Pose(14.724, 35.688, Math.toRadians(180));
//    private final Pose score2Pose = new Pose(58.9, 13.48, Math.toRadians(115));
//    private final Pose pickup2Pose = new Pose(8.98, 8.485, Math.toRadians(270));
//    private final Pose score3Pose = new Pose(59.15, 12.977, Math.toRadians(115));
//    private final Pose park Pose = new Pose(55.903, 29.948, Math.toRadians(90));
//
//
//
//    private Path scorePreload;
//    public PathChain pickup1, score2, pickup2, score3, park;
//
//    public void buildPaths() {
//
//        scorePreload = new Path(new BezierLine(startPose, score1Pose));
//        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), score1Pose.getHeading());
//
//
//        pickup1 = follower.pathBuilder()
//                .addPath(new BezierCurve(score1Pose, pickup1Pose, new Pose(44.423, 39.4315)))
//                .setLinearHeadingInterpolation(score1Pose.getHeading(), pickup1Pose.getHeading())
//                .setBrakingStart(4)
//                .setBrakingStrength(0.04)
//                .build();
//
//        score2 = follower.pathBuilder()
//                .addPath(new BezierLine(pickup1Pose, score2Pose))
//                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), score2Pose.getHeading())
//                .setBrakingStart(4)
//                .setBrakingStrength(0.04)
//                .build();
//
//        pickup2 = follower.pathBuilder()
//                .addPath(new BezierCurve(score2Pose, pickup2Pose, new Pose(9.7331, 35.938)))
//                .setLinearHeadingInterpolation(score2Pose.getHeading(), pickup2Pose.getHeading())
//                .setBrakingStart(4)
//                .setBrakingStrength(0.04)
//                .build();
//
//        score3 = follower.pathBuilder()
//                .addPath(new BezierLine(pickup2Pose, score3Pose))
//                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), score3Pose.getHeading())
//                .setBrakingStart(4)
//                .setBrakingStrength(0.04)
//                .build();
//
//
//        park = follower.pathBuilder()
//                .addPath(new BezierLine(score3Pose, parkPose))
//                .setLinearHeadingInterpolation(score3Pose.getHeading(), parkPose.getHeading())
//                .setBrakingStart(4)
//                .setBrakingStrength(0.04)
//                .build();
//
//    }
//    public void autonomousPathUpdate() {
//        switch (pathState) {
//            case 0:
//                follower.followPath(scorePreload);
//                setPathState(1);
//                break;
//            case 1:
//            /* You could check for
//            - Follower State: "if(!follower.isBusy()) {}"
//            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
//            - Robot Position: "if(follower.getPose().getX() > 36) {}"
//            */
//
//                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
//                if(!follower.isBusy()) {
//                    /* Score Preload */
//
//                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
//                    follower.followPath(pickup1,true);
//                    setPathState(2);
//                }
//                break;
//            case 2:
//                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
//                if(!follower.isBusy()) {
//                    /* Grab Sample */
//
//                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
//                    follower.followPath(score2,true);
//                    setPathState(3);
//                }
//                break;
//            case 3:
//                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
//                if(!follower.isBusy()) {
//                    /* Score Sample */
//
//                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
//                    follower.followPath(pickup2,true);
//                    setPathState(4);
//                }
//                break;
//            case 4:
//                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
//                if(!follower.isBusy()) {
//                    /* Grab Sample */
//
//                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
//                    follower.followPath(score3,true);
//                    setPathState(5);
//                }
//                break;
//            case 5:
//
//                if(!follower.isBusy()) {
//                    follower.followPath(park, true);
//                }
//                break;
//            case 8:
//                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
//                if(!follower.isBusy()) {
//                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
//                    setPathState(-1);
//                }
//                break;
//        }
//    }
//
//    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
//    public void setPathState(int pState) {
//        pathState = pState;
//        pathTimer.resetTimer();
//    }
//
//
//    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
//    @Override
//    public void loop() {
//
//        // These loop the movements of the robot, these must be called continuously in order to work
//        follower.update();
//        autonomousPathUpdate();
//
//
//    }
//
//    @Override
//    public void init() {
//        pathTimer = new Timer();
//        opmodeTimer = new Timer();
//        opmodeTimer.resetTimer();
//
//        follower = Constants.createFollower(hardwareMap);
//        buildPaths();
//        follower.setStartingPose(startPose);
//
//        setPathState(0);
//
//        telemetry.addData("path state", pathState);
//        telemetry.addData("x", follower.getPose().getX());
//        telemetry.addData("y", follower.getPose().getY());
//        telemetry.addData("heading", follower.getPose().getHeading());
//        telemetry.update();
//
//
//
//    }
//
//    public void start() {
//        opmodeTimer.resetTimer();
//    }
//
//
//}
//
