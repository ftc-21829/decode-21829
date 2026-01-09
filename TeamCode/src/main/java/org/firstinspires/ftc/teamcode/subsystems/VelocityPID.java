package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "VelocityPIDTEST")
@Config
public class VelocityPID extends OpMode {

    AllMechs r;
    public static double kV = 0.0006;
    public static double kS = 0.025;
    public static double kP = 0.0015;
    public static double targetVelocity = 0;
    private int lastLowPos = 0;
    private int lastHighPos = 0;
    private long lastTime = 0;

    @Override
    public void init() {
        r = new AllMechs(hardwareMap, gamepad1, gamepad2);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {

        lastLowPos = r.outtakeLow.getCurrentPosition();
        lastHighPos = r.outtakeHigh.getCurrentPosition();
        lastTime = System.nanoTime();
        int low = r.outtakeLow.getCurrentPosition();
        int high = r.outtakeHigh.getCurrentPosition();
        long now = System.nanoTime();

        double dt = (now - lastTime) / 1e9;
        double lowVel = (low - lastLowPos) / dt;
        double highVel = (high - lastHighPos) / dt;
        double currentVel = (lowVel + highVel) / 2.0;

        lastLowPos = low;
        lastHighPos = high;
        lastTime = now;

        double ff = feedforward(targetVelocity);
        double fb = feedback(targetVelocity, currentVel);

        double power = ff + fb;
        power = clamp(power, 0, 1);

        r.outtakeHigh.setPower(power);
        r.outtakeLow.setPower(power);


        telemetry.addData("Target Velocity", targetVelocity);
        telemetry.addData("Current Velocity", currentVel);
        telemetry.addData("Power", power);
        telemetry.update();
    }

    private double feedforward(double targetVel) {
        if (Math.abs(targetVel) < 1e-6) return 0;
        double sign = Math.signum(targetVel);
        return kS * sign + kV * targetVel;
    }

    private double feedback(double targetVel, double currentVel) {
        double error = targetVel - currentVel;
        return kP * error;
    }

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}