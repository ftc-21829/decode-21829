package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.core.command.utility.InstantCommand;
import com.rowanmcalpin.nextftc.core.command.utility.delays.Delay;
import com.rowanmcalpin.nextftc.core.command.utility.delays.WaitUntil;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Config
public class AllmechsRed {
    public DcMotorEx intake, outtakeLow, outtakeHigh, transfer;
    public Follower follower;
    public ColorSensor colorSensor;

    public Servo door, hood, buttkicker;
    public CRServo turretServo;

    public Limelight3A limelight;
    private LLResult currentResult = null;
    private boolean turretTrackingActive = false;

    public MultipleTelemetry telemetry;
    public Gamepad gamepad1;
    public Gamepad gamepad2;

    // PID & turret
    public static double turret_kP = 0.015, turret_kI = 0.0005, turret_kD = 0.004;
    public static double turret_tolerance = 1.0;
    public static double turret_max_power = 1;
    public static double turret_deadband = 0.25;
    private double integral = 0.0;
    private double lastError = 0.0;

    // ---- Shooter PID (new) ----
    private double shooter_kP = 0.005;
    private double shooter_kI = 0.0;
    private double shooter_kD = 0.00000005;

    private double shooterIntegral = 0;
    private double shooterLastError = 0;

    private int lastLowPosPID = 0;
    private int lastHighPosPID = 0;
    private long lastTimePID = 0;

    private double shooterTargetVelocity = 0;
    private double targetHoodPosition = 0;

    // servos / positions
    public static double door_open_pos = 0.75;
    public static double door_close_pos = 0.6;
    public static double butt_kicker_down = 0.88;
    public static double butt_kicker_up = 0.4675;

    public static int TRACKING_PIPELINE = 1;

    // --- shooter tuning formulas from user ---
    // hood: y = HOOD_A * HOOD_B^x
    // outtake power: y = OUTTAKE_A * OUTTAKE_B^x
    // x = distance in millimetres (from Limelight)
    private final double HOOD_A = 0.234833;
    private final double HOOD_B = 1.00578;
    private final double OUTTAKE_A = 0.39845;
    private final double OUTTAKE_B = 1.0043;

    // shooter state
    private double lastValidDistanceMM = -1.0;
    private double targetOuttakePower = 0.0;
    private boolean shooterEnabled = false;

    // rising edge detectors for user inputs
    private boolean prevDpadUp = false;
    private boolean prevDpadDown = false;
    private boolean prevLogButton = false;

    public AllmechsRed(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.telemetry = new MultipleTelemetry();

        follower = Constants.createFollower(hardwareMap);

        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        outtakeLow = hardwareMap.get(DcMotorEx.class, "outtake Low");
        outtakeHigh = hardwareMap.get(DcMotorEx.class, "outtake High");
        outtakeHigh.setDirection(DcMotorSimple.Direction.REVERSE);
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");

        turretServo = hardwareMap.get(CRServo.class, "turret");
        turretServo.setPower(0);

        door = hardwareMap.get(Servo.class, "door");
        door.setPosition(door_open_pos);
        hood = hardwareMap.get(Servo.class, "hood");
        buttkicker = hardwareMap.get(Servo.class, "buttkicker");
        buttkicker.setPosition(butt_kicker_down);

        initializeLimelight(hardwareMap);

        lastLowPosPID = outtakeLow.getCurrentPosition();
        lastHighPosPID = outtakeHigh.getCurrentPosition();
        lastTimePID = System.nanoTime();
    }

    private void initializeLimelight(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(TRACKING_PIPELINE);
    }

    public void updateLimelightData() {
        currentResult = limelight.getLatestResult();
    }

    public boolean hasValidTarget() {
        return currentResult != null && currentResult.isValid();
    }

    public double getDistanceFromLimelight(LLResult r) {
        if (r == null || !r.isValid()) return -1;
        double ty = r.getTy();
        double CAMERA_HEIGHT_MM = 270.0;
        double TARGET_HEIGHT_MM = 360.0;
        double CAMERA_ANGLE_DEG = 28.0;
        double totalAngleRad = Math.toRadians(CAMERA_ANGLE_DEG + ty);
        return (TARGET_HEIGHT_MM - CAMERA_HEIGHT_MM) / Math.tan(totalAngleRad);
    }

    public void setTurretTrackingActive(boolean active) {
        turretTrackingActive = active;
        if (!active) {
            turretServo.setPower(0);
            integral = 0.0;
            lastError = 0.0;
        }
    }

    public boolean isTurretTrackingActive() {
        return turretTrackingActive;
    }

    public void updateTurretTracking() {
        if (!turretTrackingActive || !hasValidTarget()) {
            turretServo.setPower(0);
            return;
        }

        double tx = currentResult.getTx();
        double error = tx;

        if (Math.abs(error) < turret_deadband) {
            turretServo.setPower(0);
            lastError = 0.0;
            return;
        }

        integral += error;
        integral = Math.max(-100.0, Math.min(100.0, integral));
        double derivative = error - lastError;
        double output = (turret_kP * error) + (turret_kI * integral) + (turret_kD * derivative);
        lastError = error;

        output = -output;
        output = Math.max(-turret_max_power, Math.min(turret_max_power, output));

        turretServo.setPower(output);
    }

    public boolean isTurretAligned() {
        return hasValidTarget() && Math.abs(currentResult.getTx()) < turret_tolerance;
    }

    // ------------------------ Commands ------------------------

    public Command intakeOn() { return new InstantCommand(() -> intake.setPower(1));  }
    public Command intakeOff() { return new InstantCommand(() -> intake.setPower(0)); }
    public Command doorOpen() { return new InstantCommand(() -> door.setPosition(door_open_pos)); }
    public Command doorClose() { return new InstantCommand(() -> door.setPosition(door_close_pos)); }
    public Command transferOn() { return new InstantCommand(() -> transfer.setPower(1)); }
    public Command transferOff() { return new InstantCommand(() -> transfer.setPower(0)); }
    public Command transferSlow() { return new InstantCommand(() -> transfer.setPower(0.75)); }

    // ---- OUTTAKE now uses PID flywheel + hood ----
    public Command OuttakeOn() {
        return new InstantCommand(() -> periodicShooterUpdateAndApplyPID());
    }

    public Command IntakeOut(){
        return new InstantCommand(() -> intake.setPower(-1));
    }
    public Command OuttakeOff() {
        return new ParallelGroup(
                new InstantCommand(() -> outtakeHigh.setPower(0)),
                new InstantCommand(() -> outtakeLow.setPower(0))
        );
    }
    public Command turretOn() { return new InstantCommand(() -> setTurretTrackingActive(!isTurretTrackingActive())); }
    public Command turretOff() { return new InstantCommand(() -> setTurretTrackingActive(false)); }

    public Command ButtKicker() {
        return new SequentialGroup(
                new InstantCommand(() -> buttkicker.setPosition(butt_kicker_up)),
                new Delay(0.5),
                new InstantCommand(() -> buttkicker.setPosition(butt_kicker_down))
        );
    }

    public Command transferCheck() {
        return new WaitUntil(() -> {
            double distanceCm = 100;
            if (colorSensor instanceof DistanceSensor) {
                distanceCm = ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM);
            }
            return distanceCm < 1.0;
        }).then(
                new SequentialGroup(
                        new Delay(0.3),
                        new ParallelGroup(
                                doorClose(),
                                transferOff()
                        )
                )
        );
    }

    // ------------------------ Shooter helpers ------------------------

    public double computeHoodPositionFromDistance(double distanceMM) {
        if (distanceMM <= 0) return 0.0;

        return -0.0000566307 * Math.pow(distanceMM, 2)
                + 0.0233367 * distanceMM
                -1.49896   ;

    }

    public double computeShooterTargetVelocityFromDistance(double distanceMM) {
        if (distanceMM <= 0) return 0.0;
        shooterTargetVelocity =
                0.0426503   * Math.pow(distanceMM, 2)
                        - 8.85296 * distanceMM
                        + 1675.60075; // user can replace equation
        return shooterTargetVelocity;
    }

    public void updateShooterTargetFromLimelight() {
        updateLimelightData();
        LLResult r = currentResult;
        if (r != null && r.isValid()) {
            double dmm = getDistanceFromLimelight(r);
            if (dmm > 0) {
                lastValidDistanceMM = dmm;
                targetHoodPosition = computeHoodPositionFromDistance(dmm);
                targetOuttakePower = computeShooterTargetVelocityFromDistance(dmm);
                shooterEnabled = true;
            }
        }
    }

    /** Apply PID-controlled flywheel velocity and hood position */
    public void periodicShooterUpdateAndApplyPID() {
        updateShooterTargetFromLimelight();
        if (!shooterEnabled) return;

        // PID for flywheel
        int lowPos = outtakeLow.getCurrentPosition();
        int highPos = outtakeHigh.getCurrentPosition();
        long now = System.nanoTime();
        double dt = (now - lastTimePID) / 1e9;

        double lowVel = (lowPos - lastLowPosPID) / dt;
        double highVel = (highPos - lastHighPosPID) / dt;
        double actualVel = (lowVel + highVel) / 2.0;

        lastLowPosPID = lowPos;
        lastHighPosPID = highPos;
        lastTimePID = now;

        double error = shooterTargetVelocity - actualVel;
        shooterIntegral += error * dt;
        double derivative = (error - shooterLastError) / dt;
        shooterLastError = error;

        double output = (shooter_kP * error) + (shooter_kI * shooterIntegral) + (shooter_kD * derivative);
        output = Math.max(0, Math.min(output, 1));

        outtakeLow.setPower(output);
        outtakeHigh.setPower(output);
        hood.setPosition(targetHoodPosition);
    }
}
