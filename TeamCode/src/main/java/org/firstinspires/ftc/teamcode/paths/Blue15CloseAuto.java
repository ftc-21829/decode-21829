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
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
@Configurable // Panels
public class Blue15CloseAuto extends OpMode {

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

        public PathChain ShootingPreload;
        public PathChain Intaking1;
        public PathChain Shooting1;
        public PathChain Intaking2;
        public PathChain Shooting2;
        public PathChain Intaking3;
        public PathChain Outtake3;
        public PathChain Intake4;
        public PathChain Outtake4;
        public PathChain Leave;

        public Paths(Follower follower) {
            ShootingPreload = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(19.687, 121.435), new Pose(49.856, 84.331))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(180))
                    .build();

            Intaking1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(49.856, 84.331), new Pose(18.139, 84.495))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            Shooting1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(18.139, 84.495), new Pose(51.713, 86.453))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(130))
                    .setReversed()
                    .build();

            Intaking2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(51.713, 86.453),
                                    new Pose(49.328, 43.795),
                                    new Pose(14.320, 64.707)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(180))
                    .build();

            Shooting2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(14.320, 64.707),
                                    new Pose(43.757, 71.072),
                                    new Pose(50.917, 84.066)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                    .build();

            Intaking3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(50.917, 84.066),
                                    new Pose(59.724, 28.753),
                                    new Pose(18.033, 34.740)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                    .build();

            Outtake3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(18.033, 34.740), new Pose(50.917, 84.066))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                    .build();

            Intake4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(50.917, 84.066),
                                    new Pose(12.729, 55.160),
                                    new Pose(8.406, 9.067)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(270))
                    .build();

            Outtake4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(8.406, 9.067),
                                    new Pose(39.374, 16.808),
                                    new Pose(63.927, 15.039)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(125))
                    .build();

            Leave = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(63.927, 15.039), new Pose(63.706, 39.592))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(125), Math.toRadians(90))
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