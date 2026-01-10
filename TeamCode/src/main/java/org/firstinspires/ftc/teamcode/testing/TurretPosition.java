package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
@Config
@TeleOp(name = "Turret Position (voltage)")
public class TurretPosition extends OpMode {

    private AnalogInput turretAnalog;
    private RTPAxon axon;


    // Calibration constants (match your AllMechCopy settings)
    private double minVoltage = 0.5;
    private double maxVoltage = 2.5;
    private double minRad = -Math.PI;
    private double maxRad = Math.PI;
    public static double kp = 0.00525;
    public static double ki = 0;
    public static double kd = 0.000025;
    public static double target = 0;


    @Override
    public void init() {
        CRServo servo = hardwareMap.get(CRServo.class, "turret");
        AnalogInput encoder = hardwareMap.get(AnalogInput.class, "turretAnalog");
        axon = new RTPAxon(servo, encoder);

        axon.setMaxPower(1);  // Limit max power to 50%
         // Set PID coefficients
    }

    @Override
    public void loop() {
        axon.setPidCoeffs(kp, ki, kd);
        axon.update();
        // Get current voltage

        // Display voltage (correct format - no printf-style formatting)
        axon.setTargetRotation(target * 2.3793);
        // Convert voltage to radians
        telemetry.addData("Servo Position", axon.getCurrentAngle());
        telemetry.addData("Total Rotation", axon.getTotalRotation());
        telemetry.addData("Target Rotation", axon.getTargetRotation());


        telemetry.update();
    }
}