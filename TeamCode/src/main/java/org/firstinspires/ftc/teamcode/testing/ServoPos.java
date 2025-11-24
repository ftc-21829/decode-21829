package org.firstinspires.ftc.teamcode.testing;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
@TeleOp(name = "ServoTesting", group = "testing")
public class ServoPos extends OpMode {
    public DcMotor fwLeft, fwRight, intakeFront, intakeBack;
    public Servo hood, transfer;

    public static double servo_far_pos = 0.35;
    public static double servo_middle_pos = 0.3;
    public static double servo_close_pos = 0.15;
    public static double servo_reset_pos = 1;
    public static double motor_power = 1;
    public static double transfer_pos_up = 1;
    public static double transfer_pos_down = 0;



    @Override
    public void init() {
        fwLeft = hardwareMap.get(DcMotor.class, "flyWheelLeft");
        fwRight = hardwareMap.get(DcMotor.class, "flyWheelRight");
        intakeBack = hardwareMap.get(DcMotor.class, "intakeBack");
        intakeFront = hardwareMap.get(DcMotor.class, "intakeFront");

        hood = hardwareMap.get(Servo.class, "hood");
        transfer = hardwareMap.get(Servo.class, "transfer");
        hood.setPosition(servo_reset_pos);


    }

    @Override
    public void loop() {

        if(gamepad1.dpad_left){
            fwRight.setPower(motor_power);
            fwLeft.setPower(motor_power);
        }
        if(gamepad1.dpad_up){
            hood.setPosition(servo_close_pos);
        }
        if(gamepad1.dpad_right){
            hood.setPosition(servo_middle_pos);
        }
        if(gamepad1.dpad_down){
            hood.setPosition(servo_far_pos);
        }
        if(gamepad1.square){
            hood.setPosition(servo_reset_pos);
        }
        if(gamepad1.left_bumper){
            transfer.setPosition(transfer_pos_up);
        }
        if(gamepad1.right_bumper){
            transfer.setPosition(transfer_pos_down);
        }
        if(gamepad1.circle){
            intakeFront.setPower(1);
            intakeBack.setPower(1);
        }


    }
}
