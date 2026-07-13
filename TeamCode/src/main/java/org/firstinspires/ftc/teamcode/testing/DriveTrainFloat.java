package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveTrainFloat {
    public static void setToFloatMode(HardwareMap hardwareMap) {
        try {
            DcMotorEx leftFront = hardwareMap.get(DcMotorEx.class, "front left");
            DcMotorEx leftBack = hardwareMap.get(DcMotorEx.class, "back left");
            DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class, "front right");
            DcMotorEx rightBack = hardwareMap.get(DcMotorEx.class, "back right");

            leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        } catch (Exception e) {
            // Motors might have different names in your config
            throw new RuntimeException("Failed to configure drivetrain: " + e.getMessage());
        }
    }
}
