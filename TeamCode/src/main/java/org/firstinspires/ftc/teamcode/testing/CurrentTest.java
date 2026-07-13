package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
@TeleOp(name = "CurrentTest", group = "testing")
public class CurrentTest extends OpMode {

    public DcMotorEx intake;
    public VoltageSensor battery;

    public static double CURRENT_THRESHOLD = 0;

    @Override
    public void init() {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        battery = hardwareMap.voltageSensor.iterator().next();
    }

    @Override
    public void loop() {

        double currentmA = intake.getCurrent(CurrentUnit.MILLIAMPS);
        double voltage = battery.getVoltage();

        telemetry.addData("Intake Current (mA)", currentmA);
        telemetry.addData("Battery Voltage (V)", voltage);

        if (gamepad1.circle) {
            intake.setPower(0);
        }

        if (gamepad1.cross) {
            intake.setPower(1);
        }

        telemetry.update();
    }
}
