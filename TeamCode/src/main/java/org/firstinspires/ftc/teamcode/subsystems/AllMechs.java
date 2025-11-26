package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup;
import com.rowanmcalpin.nextftc.core.command.utility.InstantCommand;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Config
public class AllMechs {
    public DcMotorEx intake, outtakeLow, outtakeHigh, transfer, frontLeft, frontRight, backLeft, backRight;
    public Follower follower;
    public ColorSensor colorSensor;
    // Continuous rotation turret servo
    public Servo door;
    public CRServo turretServo;

    // Limelight vision
    public Limelight3A limelight;
    private LLResult currentResult = null;
    private boolean turretTrackingActive = false;

    public MultipleTelemetry telemetry;
    public Gamepad gamepad1;
    public Gamepad gamepad2;

    // PID constants for TX (horizontal) tracking
    public static double turret_kP = 0.01;
    public static double turret_kI = 0.0;
    public static double turret_kD = 0.001;

    // Tracking tolerance (degrees)
    public static double turret_tolerance = 2.0;

    // Max rotation power (speed limit)
    public static double turret_max_power = 0.4;

    // PID state
    private double integral = 0.0;
    private double lastError = 0.0;

    // Limelight pipeline
    public static int TRACKING_PIPELINE = 0;

    public AllMechs(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.telemetry = new MultipleTelemetry();

        follower = Constants.createFollower(hardwareMap);

        frontLeft = hardwareMap.get(DcMotorEx.class, "front left");
        backRight = hardwareMap.get(DcMotorEx.class, "back left");
        frontRight = hardwareMap.get(DcMotorEx.class, "front right");
        backLeft = hardwareMap.get(DcMotorEx.class, "back right");
//        outtakeLow = hardwareMap.get(DcMotorEx.class, "outtake Low");
//        outtakeHigh = hardwareMap.get(DcMotorEx.class, "outtake High");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        // Initialize continuous rotation turret servo
        turretServo = hardwareMap.get(CRServo.class, "turretServo");
        door = hardwareMap.get(Servo.class, "door");
        turretServo.setPower(0);


        // Initialize Limelight
        initializeLimelight(hardwareMap);
    }

    /** Initialize Limelight 3A */
    private void initializeLimelight(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(TRACKING_PIPELINE);
    }

    /** Update Limelight data each loop */
    public void updateLimelightData() {
        currentResult = limelight.getLatestResult();
    }

    /** Check if Limelight has valid target */
    public boolean hasValidTarget() {
        return currentResult != null && currentResult.isValid();
    }

    /** Enable/disable turret tracking */
    public void setTurretTrackingActive(boolean active) {
        turretTrackingActive = active;
        if (!active) {
            // Stop turret and reset PID
            turretServo.setPower(0);
            integral = 0.0;
            lastError = 0.0;
        }
    }

    public boolean isTurretTrackingActive() {
        return turretTrackingActive;
    }

    /** PID control for turret using Limelight TX error */
    public void updateTurretTracking() {
        if (!turretTrackingActive || !hasValidTarget()) {
            turretServo.setPower(0);
            return;
        }

        // Get horizontal offset (degrees)
        double tx = currentResult.getTx();
        double error = tx;

        // PID
        integral += error;
        double derivative = error - lastError;
        double output = (turret_kP * error) + (turret_kI * integral) + (turret_kD * derivative);
        lastError = error;

        // Clamp power
        output = Math.max(-turret_max_power, Math.min(turret_max_power, output));

        // Apply power to CRServo (positive = one direction, negative = opposite)
        turretServo.setPower(output);
    }

    /** Check if turret is aligned */
    public boolean isTurretAligned() {
        if (!hasValidTarget()) return false;
        return Math.abs(currentResult.getTx()) < turret_tolerance;
    }

    /** Telemetry */
    public String getLimelightTelemetry() {
        if (!hasValidTarget()) {
            return "No target";
        }

        return String.format("TX: %.2f° | TA: %.2f%% | Pipeline: %d",
                currentResult.getTx(),
                currentResult.getTa(),
                currentResult.getPipelineIndex());
    }

    public String getTurretTelemetry() {
        return String.format("Power: %.3f | Tracking: %s | Aligned: %s",
                turretTrackingActive ? lastError * turret_kP : 0.0,
                turretTrackingActive ? "ON" : "OFF",
                isTurretAligned() ? "YES" : "NO");
    }

    /** Stop vision */
    public void closeVision() {
        if (limelight != null) {
            limelight.stop();
        }
    }

    public Command intakeOn() {
        return new InstantCommand(()-> intake.setPower(1));
    }
    public Command intakeOff(){
        return new InstantCommand(()-> intake.setPower(0));
    }
    public Command transferOn() {
        return new InstantCommand(()-> transfer.setPower(1));
    }
    public Command transferOff() {
        return new InstantCommand(()-> transfer.setPower(0));
    }
    public Command OuttakeOn() {
        return new ParallelGroup(
                new InstantCommand(()-> outtakeHigh.setPower(1)),
                new InstantCommand(()-> outtakeLow.setPower(1))
        );
    }
    public Command OuttakeOff() {
        return new ParallelGroup(
                new InstantCommand(()-> outtakeHigh.setPower(0)),
                new InstantCommand(()-> outtakeLow.setPower(0))
        );
    }
    public Command turretOn() {
        return new InstantCommand(()-> setTurretTrackingActive(!isTurretTrackingActive()));
    }
    public Command turretOff() {
        return new InstantCommand(()-> setTurretTrackingActive(false));
    }
    public Command doorOn() {
        return new InstantCommand(()-> door.setPosition(0.5));
    }
    public Command doorBack() {
        return new InstantCommand(()-> door.setPosition(0.05));
    }





}
