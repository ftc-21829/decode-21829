package org.firstinspires.ftc.teamcode.testing;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.R;

@Config
@TeleOp(name = "CurrentTest", group = "testing")
public class CurrentTest extends OpMode {
    public DcMotorEx intake, transfer;
    public static double CURRENT_THRESHOLD = 0;


    @Override
    public void init() {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    @Override
    public void loop() {


        if(gamepad1.square){
            double currentFront = intake.getCurrent(CurrentUnit.MILLIAMPS);
            telemetry.addData("CurrentFront", currentFront);
        }
        if(gamepad1.circle){
            intake.setPower(0);
            transfer.setPower(0);
        }

        if(gamepad1.cross){
            intake.setPower(1);
            transfer.setPower(1);
        }


    }
}
