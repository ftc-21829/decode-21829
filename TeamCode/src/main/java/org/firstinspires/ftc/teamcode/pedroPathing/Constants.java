package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    //change mass once robot is built (kgs)
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(10.7955)
            .forwardZeroPowerAcceleration(-57.055)
            .lateralZeroPowerAcceleration(-79.573)
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.02, 0, 0.00000, 0.75, 0.001))
            .headingPIDFCoefficients(new PIDFCoefficients(1.5, 0, 0.000000, 0.00075))
            .translationalPIDFCoefficients(new PIDFCoefficients(0.035, 0, 0.000000, 0.001))
            .centripetalScaling(0.000000075);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("front right")
            .rightRearMotorName("back right")
            .leftRearMotorName("back left")
            .leftFrontMotorName("front left")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(75.456421)
            .yVelocity(70.25123);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            //change these offsets after
            .forwardPodY(-6.6)
            .strafePodX(2.35)

            //change these if necessary (localization test)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)

            //see if yaw scalar is necessary

            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);


}
