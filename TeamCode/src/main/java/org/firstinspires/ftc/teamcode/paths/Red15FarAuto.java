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

@Autonomous(name = "Red15Far", group = "Autonomous")
@Configurable // Panels
public class Red15FarAuto extends OpMode {

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
        public PathChain Outtaking4;
        public PathChain Leave;

        public Paths(Follower follower) {
            Intaking1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(88.000, 8.000),
                                    new Pose(100.203, 36.053),
                                    new Pose(125.782, 35.438)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                    .build();

            Outtaking1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(125.782, 35.438),
                                    new Pose(90.961, 53.039),
                                    new Pose(80.610, 73.872)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                    .build();

            Intaking2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(80.610, 73.872),
                                    new Pose(109.715, 45.785),
                                    new Pose(129.844, 64.366)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                    .build();

            Outtaking2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(129.844, 64.366),
                                    new Pose(99.318, 67.463),
                                    new Pose(92.839, 85.352)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(25))
                    .build();

            Intaking3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(92.839, 85.352),
                                    new Pose(106.066, 81.608),
                                    new Pose(125.782, 83.355)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(25), Math.toRadians(0))
                    .build();

            Outtaking3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(125.782, 83.355), new Pose(92.818, 84.862))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                    .build();

            Intaking4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(92.818, 84.862),
                                    new Pose(136.922, 55.961),
                                    new Pose(136.037, 9.288)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(90))
                    .build();

            Outtaking4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(136.037, 9.288),
                                    new Pose(118.276, 13.525),
                                    new Pose(88.037, 11.942)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(60))
                    .build();

            Leave = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(88.037, 11.942), new Pose(88.575, 34.475))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(60), Math.toRadians(90))
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
                    pathState = 2;
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    /* Score Sample 1 */
                    follower.followPath(paths.Intaking2, true);
                    pathState = 3;
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    /* Grab Sample 2 */
                    follower.followPath(paths.Outtaking2, true);
                    pathState = 4;
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    /* Score Sample 2 */
                    follower.followPath(paths.Intaking3, true);
                    pathState = 5;
                }
                break;
            case 5:
                if(!follower.isBusy()) {
                    /* Grab Sample 3 */
                    follower.followPath(paths.Outtaking3, true);
                    pathState = 6;
                }
                break;
            case 6:
                if(!follower.isBusy()) {
                    /* Score Sample 3 */
                    follower.followPath(paths.Intaking4, true);
                    pathState = 7;
                }
                break;
            case 7:
                if(!follower.isBusy()) {
                    /* Grab Sample 4 */
                    follower.followPath(paths.Outtaking4, true);
                    pathState = 8;
                }
                break;
            case 8:
                if(!follower.isBusy()) {
                    /* Score Sample 4 */
                    follower.followPath(paths.Leave, true);
                    pathState = 9;
                }
                break;
            case 9:
                if(!follower.isBusy()) {
                    /* Parked - End autonomous */
                    pathState = -1;
                }
                break;
        }
        return pathState;
    }
}

