package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.AllMechs;
@TeleOp(name = "ColorSensorTHing")
public class ColorSensorTest extends OpMode {

    public AllMechs robot;
    public Telemetry telemetry;
    public ColorSensor colorsensor;

    @Override
    public void init() {
        robot = new AllMechs(hardwareMap, gamepad1, gamepad2);
        colorsensor = hardwareMap.get(ColorSensor.class, "colorSensor");

    }

    @Override
    public void loop() {

        telemetry.addData("colorsensorRed", colorsensor.red());
        telemetry.addData("colorsensorBlue", colorsensor.blue());
        telemetry.addData("colorsensorGreen", colorsensor.green());

        telemetry.update();





    }










}
