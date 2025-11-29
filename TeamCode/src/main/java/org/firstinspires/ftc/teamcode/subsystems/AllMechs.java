package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.testing.TurretPIDTest.kd;
import static org.firstinspires.ftc.teamcode.testing.TurretPIDTest.ki;
import static org.firstinspires.ftc.teamcode.testing.TurretPIDTest.kp;

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
    public DcMotorEx intake, outtakeLow, outtakeHigh, transfer;
    public Follower follower;
    public ColorSensor colorSensor;
    // Continuous rotation turret servo
    public Servo door, hood, buttkicker;
    public CRServo turretServo;

    // Limelight vision
    public Limelight3A limelight;
    private LLResult currentResult = null;
    private boolean turretTrackingActive = false;

    public MultipleTelemetry telemetry;
    public Gamepad gamepad1;
    public Gamepad gamepad2;

    // PID constants for TX (horizontal) tracking
    public static double turret_kP = 0.015;
    public static double turret_kI = 0.0005;
    public static double turret_kD = 0.004;

    // Tracking tolerance (degrees)
    public static double turret_tolerance = 1.0;

    // Max rotation power (speed limit)
    public static double turret_max_power = 1;
    public static double turret_deadband = 0.25;

    // PID state
    private double integral = 0.0;
    private double lastError = 0.0;

    public static double door_open_pos = 0.75;
    public static double door_close_pos = 0.6;
    public static double butt_kicker_down = 0.88;
    public static double butt_kicker_up = 0.4675;

    // Limelight pipeline
    public static int TRACKING_PIPELINE = 0;

    public AllMechs(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.telemetry = new MultipleTelemetry();

        follower = Constants.createFollower(hardwareMap);


        outtakeLow = hardwareMap.get(DcMotorEx.class, "outtake Low");
        outtakeHigh = hardwareMap.get(DcMotorEx.class, "outtake High");

        outtakeHigh.setDirection(DcMotorSimple.Direction.REVERSE);
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        transfer = hardwareMap.get(DcMotorEx.class, "transfer");

        // Initialize continuous rotation turret servo
        turretServo = hardwareMap.get(CRServo.class, "turret");
//        door = hardwareMap.get(Servo.class, "door");
        turretServo.setPower(0);

        door = hardwareMap.get(Servo.class, "door");
        hood = hardwareMap.get(Servo.class, "hood");
        buttkicker = hardwareMap.get(Servo.class, "buttkicker");


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

        if (Math.abs(error) < turret_deadband) {
            turretServo.setPower(0);
            lastError = 0.0;
            return;
        }

        // PID
        integral += error;
        double maxIntegral = 100.0;
        integral = Math.max(-maxIntegral, Math.min(maxIntegral, integral));
        double derivative = error - lastError;
        double output = (kp * error) + (ki * integral) + (kd * derivative);
        lastError = error;

        output = -output;

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
                turretTrackingActive ? lastError * kp : 0.0,
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
    public Command doorOpen() {
        return new InstantCommand(()-> door.setPosition(door_open_pos));
    }
    public Command doorClose() {
        return new InstantCommand(()-> door.setPosition(door_close_pos));
    }

    public Command transferOn() {
        return new InstantCommand(()-> transfer.setPower(1));
    }
    public Command transferOff() {
        return new InstantCommand(()-> transfer.setPower(0));
    }
    public Command OuttakeOn() {
        return new ParallelGroup(
                new InstantCommand(()-> outtakeHigh.setPower(0.85)),
                new InstantCommand(()-> outtakeLow.setPower(0.85))
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
    public Command buttkickerUp() {
        return new InstantCommand(()-> buttkicker.setPosition(0.4675));
    }
    public Command buttkickerDown() {
        return new InstantCommand(()-> buttkicker.setPosition(0.88));
    }






}
