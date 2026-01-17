package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "ServoTesting", group = "testing")
public class ServoPos extends OpMode {
    public DcMotor outtakeLow, outtakeHigh, intake, transfer;
    public Servo hood, door, buttkicker;
    public static double door_open_pos = 0.75;
    public static double door_close_pos = 0.6;
    public static double servo_far_pos = 0.35;
    public static double servo_middle_pos = 0.3;
    public static double servo_close_pos = 0.15;
    public static double servo_reset_pos = 1;
    public static double motor_power = 1;
    public static double transfer_pos_up = 1;
    public static double transfer_pos_down = 0;
    public static double butt_kicker_down = 0.84;
    public static double butt_kicker_up = 0.4675;



    @Override
    public void init() {
        outtakeHigh = hardwareMap.get(DcMotor.class, "outtake High");
        outtakeLow = hardwareMap.get(DcMotor.class, "outtake Low");
        intake = hardwareMap.get(DcMotor.class, "intake");
        transfer = hardwareMap.get(DcMotor.class, "transfer");
        buttkicker = hardwareMap.get(Servo.class, "buttkicker");
        door = hardwareMap.get(Servo.class, "door");
        hood = hardwareMap.get(Servo.class, "hood");



    }

    @Override
    public void loop() {

//        if(gamepad1.dpad_left){
//            outtakeLow.setPower(motor_power);
//            outtakeHigh.setPower(motor_power);
//        }
//        if(gamepad1.dpad_up){
//            hood.setPosition(servo_close_pos);
//        }
//        if(gamepad1.dpad_right){
//            hood.setPosition(servo_middle_pos);
//        }
//        if(gamepad1.dpad_down){
//            hood.setPosition(servo_far_pos);
//        }
//        if(gamepad1.square){
//            hood.setPosition(servo_reset_pos);
//        }
//        if(gamepad2.left_bumper){
//            transfer.setPower(1);
//        }
//        if(gamepad2.right_bumper){
//            transfer.setPower(0);
//        }
//
//        if(gamepad1.circle){
//            intake.setPower(1);
//        }
//        if(gamepad1.triangle){
//            door.setPosition(door_open_pos);
//        }
//        if(gamepad1.cross){
//            door.setPosition(door_close_pos);
//        }
        if(gamepad1.right_bumper) {
            buttkicker.setPosition(butt_kicker_down);
        }
        if(gamepad1.left_bumper) {
            buttkicker.setPosition(butt_kicker_up);
        }
    }
}
