//package org.firstinspires.ftc.teamcode.subsystems;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.pedropathing.follower.Follower;
//import com.qualcomm.hardware.limelightvision.LLResult;
//import com.qualcomm.hardware.limelightvision.Limelight3A;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.ColorSensor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.Gamepad;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.rowanmcalpin.nextftc.core.command.Command;
//import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup;
//import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
//import com.rowanmcalpin.nextftc.core.command.utility.InstantCommand;
//import com.rowanmcalpin.nextftc.core.command.utility.conditionals.PassiveConditionalCommand;
//import com.rowanmcalpin.nextftc.core.command.utility.delays.Delay;
//import com.rowanmcalpin.nextftc.core.command.utility.delays.WaitUntil;
//
//import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//
//@Config
//public class AllMechs {
//    public DcMotorEx intake, outtakeLow, outtakeHigh, transfer;
//    public Follower follower;
//    public ColorSensor colorSensor;
//
//    public Servo door, hood, buttkicker;
//    public CRServo turretServo;
//
//    public Limelight3A limelight;
//    private LLResult currentResult = null;
//    private boolean turretTrackingActive = false;
//
//    public MultipleTelemetry telemetry;
//    public Gamepad gamepad1;
//    public Gamepad gamepad2;
//
//    public static double turret_kP = 0.015, turret_kI = 0.0005, turret_kD = 0.004;
//    public static double turret_tolerance = 1.0;
//    public static double turret_max_power = 1;
//    public static double turret_deadband = 0.25;
//
//    private double integral = 0.0;
//    private double lastError = 0.0;
//
//    public static double door_open_pos = 0.75;
//    public static double door_close_pos = 0.6;
//    public static double butt_kicker_down = 0.88;
//    public static double butt_kicker_up = 0.4675;
//
//    public static int TRACKING_PIPELINE = 0;
//
//    public AllMechs(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {
//        this.gamepad1 = gamepad1;
//        this.gamepad2 = gamepad2;
//        this.telemetry = new MultipleTelemetry();
//
//        follower = Constants.createFollower(hardwareMap);
//
//        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
//
//        outtakeLow = hardwareMap.get(DcMotorEx.class, "outtake Low");
//        outtakeHigh = hardwareMap.get(DcMotorEx.class, "outtake High");
//        outtakeHigh.setDirection(DcMotorSimple.Direction.REVERSE);
//        intake = hardwareMap.get(DcMotorEx.class, "intake");
//        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
//
//        turretServo = hardwareMap.get(CRServo.class, "turret");
//        turretServo.setPower(0);
//
//        door = hardwareMap.get(Servo.class, "door");
//        door.setPosition(door_open_pos);
//        hood = hardwareMap.get(Servo.class, "hood");
//        buttkicker = hardwareMap.get(Servo.class, "buttkicker");
//
//        initializeLimelight(hardwareMap);
//    }
//
//    private void initializeLimelight(HardwareMap hardwareMap) {
//        limelight = hardwareMap.get(Limelight3A.class, "limelight");
//        limelight.setPollRateHz(100);
//        limelight.start();
//        limelight.pipelineSwitch(TRACKING_PIPELINE);
//    }
//
//    public void updateLimelightData() {
//        currentResult = limelight.getLatestResult();
//    }
//
//    public boolean hasValidTarget() {
//        return currentResult != null && currentResult.isValid();
//    }
//
//    public void setTurretTrackingActive(boolean active) {
//        turretTrackingActive = active;
//        if (!active) {
//            turretServo.setPower(0);
//            integral = 0.0;
//            lastError = 0.0;
//        }
//    }
//
//    public boolean isTurretTrackingActive() {
//        return turretTrackingActive;
//    }
//
//    public void updateTurretTracking() {
//        if (!turretTrackingActive || !hasValidTarget()) {
//            turretServo.setPower(0);
//            return;
//        }
//
//        double tx = currentResult.getTx();
//        double error = tx;
//
//        if (Math.abs(error) < turret_deadband) {
//            turretServo.setPower(0);
//            lastError = 0.0;
//            return;
//        }
//
//        integral += error;
//        integral = Math.max(-100.0, Math.min(100.0, integral));
//        double derivative = error - lastError;
//        double output = (turret_kP * error) + (turret_kI * integral) + (turret_kD * derivative);
//        lastError = error;
//
//        output = -output;
//        output = Math.max(-turret_max_power, Math.min(turret_max_power, output));
//
//        turretServo.setPower(output);
//    }
//
//    public boolean isTurretAligned() {
//        return hasValidTarget() && Math.abs(currentResult.getTx()) < turret_tolerance;
//    }
//
//    public Command intakeOn() { return new InstantCommand(() -> intake.setPower(1)); }
//    public Command intakeOff() { return new InstantCommand(() -> intake.setPower(0)); }
//    public Command doorOpen() { return new InstantCommand(() -> door.setPosition(door_open_pos)); }
//    public Command doorClose() { return new InstantCommand(() -> door.setPosition(door_close_pos)); }
//    public Command transferOn() {
//
//        return new InstantCommand(() -> transfer.setPower(1));
//
//    }
//
//    public Command transferCheck() {
//        return new WaitUntil(() ->
//                (
//                        // Red dominant
//                        colorSensor.red() > colorSensor.green() + 5 &&
//                                colorSensor.blue()> colorSensor.green() + 5
//
//                ) ||
//
//                        (
//                                // Green dominant
//                                colorSensor.green() > colorSensor.red() + 5 &&
//                                        colorSensor.green() > colorSensor.blue() + 5
//                        )
//        ).and(
//                doorClose(),
//                transferOff()
//        );
//
//
//    }
//
//
//    public Command transferOff() { return new InstantCommand(() -> transfer.setPower(0)); }
//    public Command OuttakeOn() { return new ParallelGroup(new InstantCommand(() -> outtakeHigh.setPower(1)), new InstantCommand(() -> outtakeLow.setPower(1))); }
//    public Command OuttakeOff() { return new ParallelGroup(new InstantCommand(() -> outtakeHigh.setPower(0)), new InstantCommand(() -> outtakeLow.setPower(0))); }
//    public Command turretOn() { return new InstantCommand(() -> setTurretTrackingActive(!isTurretTrackingActive())); }
//    public Command turretOff() { return new InstantCommand(() -> setTurretTrackingActive(false)); }
//    public Command ButtKicker() {
//        return new SequentialGroup(
//                new InstantCommand(() -> buttkicker.setPosition(butt_kicker_up)),
//                new Delay(0.5),
//                new InstantCommand(() -> buttkicker.setPosition(butt_kicker_down)));
//    }
//}
