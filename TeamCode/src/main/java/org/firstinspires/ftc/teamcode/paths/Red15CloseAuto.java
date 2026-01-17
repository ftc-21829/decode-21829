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
import org.firstinspires.ftc.teamcode.testing.DriveTrainFloat;

@Autonomous(name = "Red15Close", group = "Autonomous")
@Configurable
public class Red15CloseAuto extends OpMode {
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState;
    private AllMechCopy robot;
    private Timer actionTimer;
    private PathChain shoot1Path, pickup1Path, shoot2Path;
    private boolean shooterActive = false; // ADD THIS


    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(123.184, 123.34, Math.toRadians(36)));
        pathState = 0;
        DriveTrainFloat.setToFloatMode(hardwareMap);



        robot = new AllMechCopy(hardwareMap, gamepad1, gamepad2, follower);
        actionTimer = new Timer();

        // Build the leave path
        shoot1Path = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(123.184, 123.34, Math.toRadians(36)),
                                new Pose(86.303, 84.86, Math.toRadians(0))
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(0))
                .build();

        pickup1Path = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(86.303, 84.86, Math.toRadians(0)),
                                new Pose(125.375, 84.35, Math.toRadians(0))
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setBrakingStrength(2)
                .build();
        shoot2Path = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(125.375, 84.35, Math.toRadians(0)),
                                new Pose(86.303, 84.86, Math.toRadians(0))

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
                follower.followPath(shoot1Path, false);
                pathState = 1;
                break;
            case 1:
                // Wait for shooting to complete (adjust time as needed)
                if (actionTimer.getElapsedTimeSeconds() > 2.55) {
                    // Kick the sample out

                    CommandManager.INSTANCE.scheduleCommand(
                            new SequentialGroup(
                                    robot.OuttakeOne()
                            ));
                    if (!follower.isBusy()) {
                        pathState = 2; // Done
                        actionTimer.resetTimer();
                        shooterActive = false;
                    }
                }
                break;


            case 2:
                // Wait for butt kicker to complete
                if (actionTimer.getElapsedTimeSeconds() > 2) {
                    follower.followPath(pickup1Path);

                    CommandManager.INSTANCE.scheduleCommand(
                            new SequentialGroup(
                                    new ParallelGroup(
                                            robot.transferOn(),
                                            robot.intakeOn(),
                                            robot.transferCheck(),
                                            robot.doorOpen(),
                                            new WaitUntil(()->{
                                                double current = robot.intake.getCurrent(CurrentUnit.MILLIAMPS);
                                                return current> 6550;
                                            }).then(
                                                    new ParallelGroup(
                                                            robot.intakeOff(),
                                                            robot.transferOff()
                                                    )

                                            )
                                    )
                            )
                    );
                    if (!follower.isBusy()) {
                        pathState = 3;
                        shooterActive = true;
                        actionTimer.resetTimer();
                    }
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

