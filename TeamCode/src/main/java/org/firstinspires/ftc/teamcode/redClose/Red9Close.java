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

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.AllMechCopy;
import org.firstinspires.ftc.teamcode.subsystems.PoseStorage;
import org.firstinspires.ftc.teamcode.subsystems.TurretPoseStorage;
import org.firstinspires.ftc.teamcode.testing.DriveTrainFloat;

@Autonomous(name = "Red9Close", group = "Autonomous")
@Configurable
public class Red9Close extends OpMode {
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState;
    private AllMechCopy robot;
    private Timer actionTimer;
    private PathChain shoot1Path, pickup1Path, shoot2Path, pickup2Path, shoot3Path, pickup3Path, shoot4Path, leavePath;
    private boolean shooterActive = false; // ADD THIS


    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(122.33, 122.855, Math.toRadians(36)));
        pathState = 0;
        DriveTrainFloat.setToFloatMode(hardwareMap);
        PoseStorage.x = 122.33;
        PoseStorage.y = 122.855;
        PoseStorage.heading = Math.toRadians(36);


        robot = new AllMechCopy(hardwareMap, gamepad1, gamepad2, follower);
        robot.UpdateTarget(144,144);


        TurretPoseStorage.autoEndTurretAngle = robot.getRotation();

        actionTimer = new Timer();

        // Build the leave path
        shoot1Path = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(122.33, 122.855, Math.toRadians(36)),
                                new Pose(90.296, 85.595, Math.toRadians(0))
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(0))
                .build();

        pickup1Path = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(90.296, 85.595, Math.toRadians(0)),
                                new Pose(117.63, 86.124, Math.toRadians(0))
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setVelocityConstraint(10)
                .build();

        shoot2Path = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(117.63, 86.124, Math.toRadians(0)),
                                new Pose(90.296, 85.595, Math.toRadians(0))
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        pickup2Path = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(90.296, 85.595, Math.toRadians(0)),
                                new Pose(83.4021, 56.84113),
                                new Pose(120.641, 55.263, Math.toRadians(0))
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setVelocityConstraint(10)
                .build();

        shoot3Path = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(120.641, 55.263, Math.toRadians(0)),
                                new Pose(100.4364, 62.6470),
                                new Pose(90.296, 85.595, Math.toRadians(0))
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();


        leavePath = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(90.296, 85.595, Math.toRadians(0)),
                                new Pose(96, 72, Math.toRadians(90))
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
                robot.UpdateTarget(138,146);

                follower.followPath(shoot1Path, false);

                pathState = 1;
                break; // ✅

            case 1:
                // Wait for path to complete AND timer
                if (!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > 2.95) {
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
                    robot.UpdateTarget(140,150);

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
                            robot.intakeAndTransferAuto()
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
                    robot.UpdateTarget(148, 150);

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
                if (actionTimer.getElapsedTimeSeconds() > 1.5) {
                    follower.followPath(leavePath);
                    pathState = 9; // Done
                }
                break; // ✅
            case 9:
                // Wait for final path to finish
                if (!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > 2.5) {

                    // Schedule turret zeroing

                    pathState = 10; // Done
                }
                break;

            case 10:
                // Autonomous complete, do nothing
                break;
        }



    }
}



