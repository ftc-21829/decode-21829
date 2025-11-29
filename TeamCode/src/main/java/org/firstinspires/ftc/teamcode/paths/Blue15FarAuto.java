package org.firstinspires.ftc.teamcode.paths;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup;
import com.rowanmcalpin.nextftc.core.command.utility.InstantCommand;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.AllMechs;

@Autonomous(name = "Blue15Far", group = "Autonomous")
@Configurable // Panels
public class Blue15FarAuto extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine

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
                                    new Pose(37.685, 38.932),
                                    new Pose(18.218, 35.438)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                    .build();

            Outtaking1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(18.218, 35.438),
                                    new Pose(46.170, 43.674),
                                    new Pose(63.390, 73.872)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(145))
                    .build();

            Intaking2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(63.390, 73.872),
                                    new Pose(40.430, 48.915),
                                    new Pose(14.475, 64.388)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(180))
                    .build();

            Outtaking2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(14.475, 64.388),
                                    new Pose(45.421, 70.627),
                                    new Pose(51.161, 85.352)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(145))
                    .build();

            Intaking3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(51.161, 85.352),
                                    new Pose(37.934, 81.608),
                                    new Pose(18.218, 83.355)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(180))
                    .build();

            Outtaking3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(18.218, 83.355), new Pose(50.912, 85.352))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(145))
                    .build();

            Intaking4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(50.912, 85.352),
                                    new Pose(9.983, 41.428),
                                    new Pose(7.737, 8.485)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(270))
                    .build();

            Outtake4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(7.737, 8.485),
                                    new Pose(41.428, 14.974),
                                    new Pose(55.154, 11.730)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(90))
                    .build();

            Leave = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(55.154, 11.730), new Pose(55.404, 28.201))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(90))
                    .build();
        }
    }

    public int autonomousPathUpdate() {
        // Add your state machine Here
        // Access paths with paths.pathName
        // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine
        return pathState;
    }
}
