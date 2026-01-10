package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

@TeleOp(name = "Turret Position (voltage)")
public class TurretPosition {

    private AnalogInput turretAnalog;
    private MultipleTelemetry telemetry;

    // Calibration constants (match your AllMechCopy settings)
    private double minVoltage = 0.5;
    private double maxVoltage = 2.5;
    private double minRad = -Math.PI;
    private double maxRad = Math.PI;

    public TurretPosition(HardwareMap hardwareMap, MultipleTelemetry telemetry) {
        this.telemetry = telemetry;
        this.turretAnalog = hardwareMap.get(AnalogInput.class, "turretAnalog");
    }

    /** Call periodically to test turret voltage and angle */
    public void update() {
        double voltage = turretAnalog.getVoltage();
        telemetry.addData("Turret Voltage (V)", "%.3f", voltage);

        // Convert voltage to radians
        double clamped = Math.max(minVoltage, Math.min(maxVoltage, voltage));
        double ratio = (clamped - minVoltage) / (maxVoltage - minVoltage);
        double angleRad = minRad + ratio * (maxRad - minRad);

        // Normalize to [-pi, pi]
        angleRad = Math.atan2(Math.sin(angleRad), Math.cos(angleRad));
        telemetry.addData("Turret Angle (deg)", "%.2f", Math.toDegrees(angleRad));

        telemetry.update();
    }
}
