package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.rowanmcalpin.nextftc.core.command.CommandManager;

@TeleOp(name = "Flywheel PID Tester (Distance + PID)", group = "Testing")
public class FlywheelDistanceLogger extends OpMode {

    AllMechCopy r;

    private DcMotorEx outtakeLow;
    private DcMotorEx outtakeHigh;
    private Servo hood;
    private Limelight3A limelight;

    // Target velocity
    private double targetVelocity = 0;

    // PID
    private double kP = 0.003;
    private double kI = 0.0;
    private double kD = 0.00000005;

    private double integral = 0;
    private double lastError = 0;

    private double hoodPos = 0.2;

    private boolean lastInc = false;
    private boolean lastDec = false;

    private int lastLowPos = 0;
    private int lastHighPos = 0;
    private long lastTime = 0;

    @Override
    public void init() {

        // ---- SAFE HARDWARE MAPPING ----
        try {
            outtakeLow = hardwareMap.get(DcMotorEx.class, "outtake Low");
            outtakeHigh = hardwareMap.get(DcMotorEx.class, "outtake High");
            hood = hardwareMap.get(Servo.class, "hood");
        } catch (Exception e) {
            telemetry.addData("HARDWARE ERROR", "Name mismatch in configuration!");
            telemetry.update();
            return;
        }


        outtakeHigh.setDirection(DcMotor.Direction.REVERSE);

        outtakeLow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outtakeHigh.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        hood.setPosition(hoodPos);
        r = new AllMechCopy(hardwareMap, gamepad1, gamepad2);

        // Limelight (safe)
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.pipelineSwitch(0);
            limelight.start();
        } catch (Exception ignored) {}

        // Initialize velocity tracking
        lastLowPos = outtakeLow.getCurrentPosition();
        lastHighPos = outtakeHigh.getCurrentPosition();
        lastTime = System.nanoTime();

        // Confirm motors

        telemetry.addLine("INIT OK — Motors spinning lightly.");
        telemetry.update();
    }


    // ----------------------------------------------------------------------
    //  NEW FUNCTION — gets distance using vertical angle (ty)
    // ----------------------------------------------------------------------
    public double getDistanceFromLimelight(LLResult r) {
        if (r == null || !r.isValid()) return -1;

        double ty = r.getTy();

        double CAMERA_HEIGHT_MM = 270.0;
        double TARGET_HEIGHT_MM = 360.0;
        double CAMERA_ANGLE_DEG = 28.0;

        double totalAngleRad = Math.toRadians(CAMERA_ANGLE_DEG + ty);

        return (TARGET_HEIGHT_MM - CAMERA_HEIGHT_MM) / Math.tan(totalAngleRad);
    }
    // ----------------------------------------------------------------------


    @Override
    public void loop() {

        // ---- READ LIMELIGHT DISTANCE ----
        LLResult result = limelight != null ? limelight.getLatestResult() : null;
        double distance = getDistanceFromLimelight(result);



        // ---- VELOCITY INCREASE ----
        boolean inc = gamepad1.right_bumper;
        if (inc && !lastInc) {
            targetVelocity += 50;
            gamepad1.rumble(200);
        }
        lastInc = inc;

        // ---- VELOCITY DECREASE ----
        boolean dec = gamepad1.left_bumper;
        if (dec && !lastDec) {
            targetVelocity -= 50;
            if (targetVelocity < 0) targetVelocity = 0;
            gamepad1.rumble(200);
        }
        lastDec = dec;

        // ---- ACTUAL VELOCITY ----
        int low = outtakeLow.getCurrentPosition();
        int high = outtakeHigh.getCurrentPosition();
        long now = System.nanoTime();

        double dt = (now - lastTime) / 1e9;
        double lowVel = (low - lastLowPos) / dt;
        double highVel = (high - lastHighPos) / dt;
        double actualVel = (lowVel + highVel) / 2.0;

        lastLowPos = low;
        lastHighPos = high;
        lastTime = now;

        // ---- PID ----
        double error = targetVelocity - actualVel;
        integral += error * dt;
        double derivative = (error - lastError) / dt;
        lastError = error;

        double output = kP * error + kI * integral + kD * derivative;
        output = Math.max(0, Math.min(output, 1));

        outtakeLow.setPower(output);
        outtakeHigh.setPower(output);

        // ---- HOOD ----
        if (gamepad1.dpad_up) hoodPos += 0.01;
        if (gamepad1.dpad_down) hoodPos -= 0.01;

        if(gamepad1.cross){
            CommandManager.INSTANCE.scheduleCommand(
                    r.transferOn()
            );
        }
        if(gamepad1.square){
            CommandManager.INSTANCE.scheduleCommand(
                    r.transferOff()
            );
        }
        if(gamepad1.triangle){
            CommandManager.INSTANCE.scheduleCommand(
                    r.intakeOn()
            );
        }
        if(gamepad1.circle){
            CommandManager.INSTANCE.scheduleCommand(
                    r.intakeOff()
            );
        }

        hoodPos = Math.max(0, Math.min(1, hoodPos));
        hood.setPosition(hoodPos);


        CommandManager.INSTANCE.run();

        // ---- TELEMETRY ----
        telemetry.addData("Target Vel", targetVelocity);
        telemetry.addData("Actual Vel", actualVel);
        telemetry.addData("Power", output);
        telemetry.addData("Hood Pos", hoodPos);

        // NEW TELEMETRY
        telemetry.addData("Limelight Valid?", result != null && result.isValid());
        telemetry.addData("Distance (mm)", distance);

        telemetry.update();
    }

    @Override
    public void stop() {
        if (outtakeLow != null) outtakeLow.setPower(0);
        if (outtakeHigh != null) outtakeHigh.setPower(0);
        if (limelight != null) limelight.stop();
    }
}
