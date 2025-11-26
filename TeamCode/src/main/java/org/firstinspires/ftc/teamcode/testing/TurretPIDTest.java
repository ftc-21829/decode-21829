package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "PIDTurret")
public class TurretPIDTest extends OpMode {

    public DcMotor fwBottom, fwTop;
    public Servo hood;
    public Gamepad previousGamepad1, currentGamepad1;

    public static double kp = 0.02;
    public static double ki = 0.0005;  // Small integral to eliminate steady-state error
    public static double kd = 0.004;
    public static double turret_tolerance = 1.0;

    // Max rotation power (speed limit)
    public static double turret_max_power = 0.85;
    private double integral = 0.0;
    private double lastError = 0.0;

    // Limelight pipeline
    public static int TRACKING_PIPELINE = 0;

    public CRServo turretServo;
    public Limelight3A limelight;
    private LLResult currentResult = null;
    private boolean turretTrackingActive = false;
    public static double turret_deadband = 0.25;  // degrees





    @Override
    public void init() {
        turretServo = hardwareMap.get(CRServo.class, "turret");
        turretServo.setPower(0);

        // Initialize Limelight
        initializeLimelight(hardwareMap);

        previousGamepad1 = new Gamepad();
        currentGamepad1 = new Gamepad();
    }
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

    @Override
    public void loop() {
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);

        updateLimelightData();
        boolean hasTarget = hasValidTarget();
        LLResult result = hasTarget ? limelight.getLatestResult() : null;

        // ========== TURRET TRACKING TOGGLE ==========
        if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
            setTurretTrackingActive(!isTurretTrackingActive());
        }

        // Disable tracking if target is lost
//        if (!hasTarget && isTurretTrackingActive()) {
//            setTurretTrackingActive(false);
//        }

        // ========== UPDATE TURRET ==========
        updateTurretTracking();

        // ========== TELEMETRY ==========
        telemetry.addLine("=== LIMELIGHT STATUS ===");
        if (hasTarget && result != null) {
            telemetry.addData("Status", "TARGET LOCKED ✓");
            telemetry.addData("Info", getLimelightTelemetry());
        } else {
            telemetry.addData("Status", "NO TARGET");
        }
        telemetry.addLine();
        telemetry.addLine("=== TURRET STATUS ===");
        telemetry.addData("Info", getTurretTelemetry());
        if (hasTarget) {
            telemetry.addData("Control",
                    isTurretTrackingActive() ? "🟢 AUTO TRACKING (R1=OFF)" : "⚪ MANUAL (R1=ON)");
        } else {
            telemetry.addData("Control", "⚠️ NO TARGET DETECTED");
        }

        telemetry.update();
    }
}
