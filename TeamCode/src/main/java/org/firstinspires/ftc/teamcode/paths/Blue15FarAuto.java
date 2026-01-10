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
                                    new Pose(56.000+24, 8.000),
                                    new Pose(37.685+24, 38.932),
                                    new Pose(20.218+24, 35.438)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                    .build();

            Outtaking1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(20.218+24, 35.438),
                                    new Pose(46.170+24, 43.674),
                                    new Pose(63.390+24, 73.872)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(145))
                    .build();

            Intaking2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(63.390+24, 73.872),
                                    new Pose(40.430+24, 48.915),
                                    new Pose(20.475+14, 62.388) //67.388
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(180))
                    .build();

            Outtaking2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(20.475+14, 62.388),
                                    new Pose(45.421+24, 70.627),
                                    new Pose(51.161+24, 85.352)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(145))
                    .build();

            Intaking3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(51.161+24, 85.352),
                                    new Pose(37.934+24, 81.608),
                                    new Pose(20.218+24, 81.355)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(180))
                    .build();

            Outtaking3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(20.218+24, 81.355), new Pose(50.912+24, 85.352))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(145))
                    .build();

            Intaking4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(50.912+24, 85.352),
                                    new Pose(9.983+24, 41.428),
                                    new Pose(7.737+24, 8.485)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(270))
                    .build();

            Outtake4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(7.737+24, 8.485),
                                    new Pose(41.428+24, 14.974),
                                    new Pose(55.154+24, 11.730)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(90))
                    .build();

            Leave = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(55.154+24, 11.730), new Pose(55.404+24, 28.201))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(90))
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
                    follower.followPath(paths.Outtake4, true);
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
