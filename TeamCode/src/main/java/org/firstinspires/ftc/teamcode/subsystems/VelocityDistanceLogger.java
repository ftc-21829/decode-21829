package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.rowanmcalpin.nextftc.core.command.CommandManager;

@TeleOp(name = "VelocityPIDTest", group = "Testing")
public class VelocityDistanceLogger extends OpMode {

    AllMechs r;

    private DcMotorEx outtakeLow;
    private DcMotorEx outtakeHigh;
    private Servo hood;
    private Limelight3A limelight;

    // Target velocity
    private double targetVelocity = 0;

    // Velocity PID Constants (from VelocityPID class)
    private double kV = 0.0006;
    private double kS = 0.025;
    private double kP = 0.0015;

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
        r = new AllMechs(hardwareMap, gamepad1, gamepad2);
        r.follower.setStartingPose(new Pose(72,72,Math.toRadians(90)));

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

        telemetry.addLine("INIT OK — Motors spinning lightly.");
        telemetry.update();
    }

    // ----------------------------------------------------------------------
    //  Gets distance using vertical angle (ty)
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
    //  Feedforward calculation (from VelocityPID)
    // ----------------------------------------------------------------------
    private double feedforward(double targetVel) {
        if (Math.abs(targetVel) < 1e-6) return 0;
        double sign = Math.signum(targetVel);
        return kS * sign + kV * targetVel;
    }

    // ----------------------------------------------------------------------
    //  Feedback calculation (from VelocityPID)
    // ----------------------------------------------------------------------
    private double feedback(double targetVel, double currentVel) {
        double error = targetVel - currentVel;
        return kP * error;
    }

    private double distance(double x, double y){
        return Math.pow((Math.pow(x,2)+Math.pow((144-y),2)), 0.5);

    }

    // ----------------------------------------------------------------------
    //  Clamp utility (from VelocityPID)
    // ----------------------------------------------------------------------
    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    @Override
    public void loop() {

        // ---- READ LIMELIGHT DISTANCE ----
        r.follower.update();
        LLResult result = limelight != null ? limelight.getLatestResult() : null;
        double distance = getDistanceFromLimelight(result);

        // ---- VELOCITY INCREASE ----
        boolean inc = gamepad1.right_bumper;
        if (gamepad1.rightBumperWasPressed()) {
            targetVelocity += 50;
            gamepad1.rumble(200);
        }
        lastInc = inc;

        // ---- VELOCITY DECREASE ----
        boolean dec = gamepad1.left_bumper;
        if (gamepad1.leftBumperWasPressed()) {
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
        double currentVel = (lowVel + highVel) / 2.0;

        lastLowPos = low;
        lastHighPos = high;
        lastTime = now;

        // ---- VELOCITY PID CONTROL (from VelocityPID) ----
        double ff = feedforward(targetVelocity);
        double fb = feedback(targetVelocity, currentVel);

        double power = ff + fb;
        power = clamp(power, 0, 1);

        outtakeLow.setPower(power);
        outtakeHigh.setPower(power);

        // ---- HOOD ----
        if (gamepad1.dpadUpWasPressed()) hoodPos += 0.01;
        if (gamepad1.dpadDownWasPressed()) hoodPos -= 0.01;

        if(gamepad1.crossWasPressed()){
            CommandManager.INSTANCE.scheduleCommand(
                    r.transferfull()
            );
        }
        if(gamepad1.squareWasPressed()){
            CommandManager.INSTANCE.scheduleCommand(
                    r.transferOff()
            );
        }
        if(gamepad1.triangleWasPressed()){
            CommandManager.INSTANCE.scheduleCommand(
                    r.intakeOn()
            );
        }
        if(gamepad1.circleWasPressed()){
            CommandManager.INSTANCE.scheduleCommand(
                    r.intakeOff()
            );
        }

        hoodPos = clamp(hoodPos, 0, 0.7);
        hood.setPosition(hoodPos);
        Pose robotPose = r.follower.getPose();

        CommandManager.INSTANCE.run();

        // ---- TELEMETRY ----
        telemetry.addData("Target Vel", targetVelocity);
        telemetry.addData("Current Vel", currentVel);
        telemetry.addData("Power", power);
        telemetry.addData("Feedforward", ff);
        telemetry.addData("Feedback", fb);
        telemetry.addData("Hood Pos", hoodPos);
        telemetry.addData("Limelight Valid?", result != null && result.isValid());
        telemetry.addData("Distance (mm)", distance);
        telemetry.addData("Robot X: ", robotPose.getX());
        telemetry.addData("Robot Y: ", robotPose.getY());

        telemetry.addData("Distance via odometry: ", distance(robotPose.getX(), robotPose.getY()));

        telemetry.update();
    }

    @Override
    public void stop() {
        if (outtakeLow != null) outtakeLow.setPower(0);
        if (outtakeHigh != null) outtakeHigh.setPower(0);
        if (limelight != null) limelight.stop();
    }
}