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

@Autonomous(name = "Red15Close", group = "Autonomous")
@Configurable // Panels
public class Red15CloseAuto extends OpMode {

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
        public PathChain Outtaking1;
        public PathChain Intaking2;
        public PathChain Outtaking2;
        public PathChain Intaking3;
        public PathChain Outtaking3;
        public PathChain Intaking4;
        public PathChain Outtaking4;
        public PathChain Leave;

        public Paths(Follower follower) {
            ShootingPreload = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(127.823, 118.807), new Pose(89.370, 82.475))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(45))
                    .build();

            Intaking1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(89.370, 82.475),
                                    new Pose(103.079, 80.956),
                                    new Pose(125.862, 83.389)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                    .build();

            Outtaking1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(125.862, 83.389), new Pose(90.431, 82.740))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                    .build();

            Intaking2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(90.431, 82.740),
                                    new Pose(97.770, 44.016),
                                    new Pose(130.286, 64.587)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                    .build();

            Outtaking2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(130.286, 64.587),
                                    new Pose(101.039, 62.586),
                                    new Pose(89.901, 83.536)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                    .build();

            Intaking3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(89.901, 83.536),
                                    new Pose(80.354, 28.110),
                                    new Pose(126.762, 34.740)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                    .build();

            Outtaking3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(126.762, 34.740),
                                    new Pose(98.652, 46.144),
                                    new Pose(89.901, 82.740)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                    .build();

            Intaking4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(89.901, 82.740),
                                    new Pose(128.884, 48.000),
                                    new Pose(136.480, 8.624)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(270))
                    .build();

            Outtaking4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(136.480, 8.624),
                                    new Pose(114.360, 16.145),
                                    new Pose(86.047, 12.606)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(60))
                    .build();

            Leave = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(86.047, 12.606), new Pose(86.453, 32.884))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(60), Math.toRadians(90))
                    .build();
        }
    }

    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(paths.ShootingPreload);
                pathState = 1;
                break;
            case 1:
                if(!follower.isBusy()) {
                    /* Grab Sample 1 */
                    follower.followPath(paths.Intaking1, true);

                }
                pathState = 2;
                break;
            case 2:
                if(!follower.isBusy()) {
                    /* Grab Sample 1 */
                    follower.followPath(paths.Outtaking1, true);
                    pathState = 3;
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    /* Score Sample 1 */
                    follower.followPath(paths.Intaking2, true);
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
                    follower.followPath(paths.Outtaking4, true);
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
}

