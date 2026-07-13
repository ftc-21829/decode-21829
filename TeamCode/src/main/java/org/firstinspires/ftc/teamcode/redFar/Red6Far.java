package org.firstinspires.ftc.teamcode.redFar;

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

@Autonomous(name = "Red6Far", group = "Autonomous")
@Configurable
public class Red6Far extends OpMode {
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState;
    private AllMechCopy robot;
    private Timer actionTimer;
    private PathChain shoot1Path, pickup1Path, shoot2Path, pickup2Path, shoot3Path, pickup3Path, shoot0Path, leavePath;
    private boolean shooterActive = false; // ADD THIS


    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(88, 8, Math.toRadians(90)));
        pathState = 0;
        DriveTrainFloat.setToFloatMode(hardwareMap);
        PoseStorage.x = 88;
        PoseStorage.y = 8;
        PoseStorage.heading = Math.toRadians(90);


        robot = new AllMechCopy(hardwareMap, gamepad1, gamepad2, follower);
        TurretPoseStorage.autoEndTurretAngle = robot.getRotation();

        actionTimer = new Timer();

        shoot0Path = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(88, 8, Math.toRadians(90)),
                                new Pose(88, 10.5, Math.toRadians(90))
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();
        // Build the leave path
        pickup1Path = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(8, 10.5, Math.toRadians(90)),
                                new Pose(88.5083, 34.407),
                                new Pose(125.154, 35.672, Math.toRadians(0))
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                .build();

        shoot1Path = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(125.154, 35.672, Math.toRadians(0)),
                                new Pose(93.3534,43),
                                new Pose(88.656, 73.464, Math.toRadians(0))
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setVelocityConstraint(10)
                .build();

        leavePath = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(88.656, 73.464, Math.toRadians(0)),
                                new Pose(90, 12, Math.toRadians(90))
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
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
                robot.UpdateTarget(141, 152);

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
                if (actionTimer.getElapsedTimeSeconds() > 2.5) {
                    // Start pickup path
                    follower.followPath(pickup1Path, true);

                    // Schedule intake ONCE
                    CommandManager.INSTANCE.scheduleCommand(
                            robot.intakeAndTransferAuto()
                    );



                    actionTimer.resetTimer();
                    pathState = 3;
                }
                break; // ✅

            case 3:
                // Wait for pickup path to complete AND intake to finish
                if (!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > 2.5) {
                    follower.followPath(shoot1Path, false);
                    robot.UpdateTarget(144,152);


                    actionTimer.resetTimer();

                    pathState = 4;
                }
                break; // ✅

            case 4:
                // Wait for path to complete AND shooter to spin up
                if (!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > 2.55) {
                    // Start shoot path
                    CommandManager.INSTANCE.scheduleCommand(
                            robot.OuttakeOne()

                    );

                    actionTimer.resetTimer();
                    pathState = 5;
                }
                break; // ✅

            case 5:
                if (actionTimer.getElapsedTimeSeconds() > 1.75) {
                    follower.followPath(leavePath);
                    pathState = 6; // Done
                }
                break; // ✅
            case 6:
                // Wait for final path to finish
                if (!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > 2.5) {

                    // Schedule turret zeroing

                    pathState = 7; // Done
                }
                break;

            case 7:
                // Autonomous complete, do nothing
                break;
        }
    }
}



