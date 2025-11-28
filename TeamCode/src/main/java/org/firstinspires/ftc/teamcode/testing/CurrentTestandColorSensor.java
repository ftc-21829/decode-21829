package org.firstinspires.ftc.teamcode.testing;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.R;

@Config
@TeleOp(name = "CurrentTest&ColorSensor", group = "testing")
public class CurrentTestandColorSensor extends OpMode {
    public DcMotorEx intake;
    public ColorSensor colorSensorFront, colorSensorBack;
    public static double CURRENT_THRESHOLD = 0;


    @Override
    public void init() {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        colorSensorFront = hardwareMap.get(ColorSensor.class, "colorSensorFront");
        colorSensorBack = hardwareMap.get(ColorSensor.class, "colorSensorBack");
    }

    @Override
    public void loop() {

        // Get distance in different units
        double distanceCm = ((DistanceSensor) colorSensorBack).getDistance(DistanceUnit.CM);
        double distanceInch = ((DistanceSensor) colorSensorBack).getDistance(DistanceUnit.INCH);
        double currentFront = intake.getCurrent(CurrentUnit.MILLIAMPS);
        telemetry.addData("Back Sensor Red", colorSensorBack.red());
        telemetry.addData("Back Sensor Green", colorSensorBack.green());
        telemetry.addData("Back Sensor Blue", colorSensorBack.blue());
        telemetry.addData("Back Sensor Alpha", colorSensorBack.alpha());
        telemetry.addData("ColorSensorBack distance CM", distanceCm);
        telemetry.addData("ColorSensorBack distance IN", distanceInch);
        telemetry.update();


        if(gamepad1.circle){
            intake.setPower(0);
        }

        if(gamepad1.cross){
            intake.setPower(1);
        }


    }
}
