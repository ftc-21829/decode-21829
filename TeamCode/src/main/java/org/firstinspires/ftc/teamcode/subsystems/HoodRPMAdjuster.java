package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.TreeMap;

public class HoodRPMAdjuster {

    private TreeMap<Double, ShooterSettings> lookupTable;

    private DcMotorEx flyWheelLeft;
    private DcMotorEx flyWheelRight;
    private Servo hoodServo;

    // Shooter performance compensation
    private double performanceAdjustmentFactor = 1.0;
    private double performanceFeedbackGain = 0.05;
    private double lastTargetRPM = 0;
    private int samplesForAdjustment = 0;

    // Limelight camera geometry (EDIT FOR YOUR ROBOT)
    private static final double CAMERA_HEIGHT_MM = 270;   // LL height from ground
    private static final double TARGET_HEIGHT_MM = 360;   // Height of AprilTag center
    private static final double CAMERA_ANGLE_DEG = 28;    // Upwards tilt angle of LL

    // Bounds
    private static final double MIN_DISTANCE_MM = 300;
    private static final double MAX_DISTANCE_MM = 3000;

    private static final double HOOD_MIN_POS = 0.1;
    private static final double HOOD_MAX_POS = 0.325;
    private static final double MIN_HOOD_ANGLE = 0.0;
    private static final double MAX_HOOD_ANGLE = 45.0;

    private static final double TICKS_PER_REV = 28.0;

    public HoodRPMAdjuster(DcMotorEx left, DcMotorEx right, Servo hood) {
        this.flyWheelLeft = left;
        this.flyWheelRight = right;
        this.hoodServo = hood;

        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        initializeLookupTable();
    }

    private void initializeLookupTable() {
        lookupTable = new TreeMap<>();

        lookupTable.put(600.0, new ShooterSettings(4000, 10));
        lookupTable.put(900.0, new ShooterSettings(4300, 15));
        lookupTable.put(1200.0, new ShooterSettings(4700, 20));
        lookupTable.put(1500.0, new ShooterSettings(5000, 25));
        lookupTable.put(1800.0, new ShooterSettings(5050, 30));
        lookupTable.put(2100.0, new ShooterSettings(5200, 35));
        lookupTable.put(2400.0, new ShooterSettings(5500, 40));
        lookupTable.put(2700.0, new ShooterSettings(6000, 42));
    }

    // ------------------------ LIMELIGHT DISTANCE ------------------------

    public double getDistanceFromLimelight(LLResult result) {
        if (result == null || !result.isValid()) return -1;

        double ty = result.getTy();        // vertical offset
        double totalAngleRad = Math.toRadians(CAMERA_ANGLE_DEG + ty);

        double distance = (TARGET_HEIGHT_MM - CAMERA_HEIGHT_MM) / Math.tan(totalAngleRad);
        return distance;
    }

    // ------------------------ SETTINGS FROM LIMELIGHT ------------------------

    public ShooterSettings getSettingsFromLimelight(LLResult result) {
        double dist = getDistanceFromLimelight(result);
        if (dist <= 0) return null;
        return getSettingsForDistance(dist);
    }

    public ShooterSettings getSettingsForDistance(double dist) {
        if (dist < MIN_DISTANCE_MM || dist > MAX_DISTANCE_MM) return null;

        if (dist <= lookupTable.firstKey()) return lookupTable.firstEntry().getValue();
        if (dist >= lookupTable.lastKey()) return lookupTable.lastEntry().getValue();

        Double lower = lookupTable.floorKey(dist);
        Double upper = lookupTable.ceilingKey(dist);

        ShooterSettings lo = lookupTable.get(lower);
        ShooterSettings hi = lookupTable.get(upper);

        double ratio = (dist - lower) / (upper - lower);

        double rpm = lo.rpm + (hi.rpm - lo.rpm) * ratio;
        double angle = lo.hoodAngleDegrees + (hi.hoodAngleDegrees - lo.hoodAngleDegrees) * ratio;

        return new ShooterSettings(rpm, angle);
    }

    // ------------------------ APPLY SETTINGS ------------------------

    public boolean applyLimelightShot(LLResult result) {
        ShooterSettings s = getSettingsFromLimelight(result);
        if (s == null) return false;

        double adjustedRPM = s.rpm * performanceAdjustmentFactor;

        lastTargetRPM = adjustedRPM;
        updatePerformanceAdjustment();

        setFlywheelRPM(adjustedRPM);
        setHoodAngle(s.hoodAngleDegrees);

        return true;
    }

    private void updatePerformanceAdjustment() {
        if (samplesForAdjustment < 10) {
            samplesForAdjustment++;
            return;
        }

        double actual = getCurrentRPM();

        if (lastTargetRPM > 0) {
            double ratio = actual / lastTargetRPM;
            double correction = 1.0 / ratio;

            performanceAdjustmentFactor =
                    performanceAdjustmentFactor * (1 - performanceFeedbackGain) +
                            correction * performanceFeedbackGain;

            performanceAdjustmentFactor =
                    Math.max(0.8, Math.min(1.2, performanceAdjustmentFactor));
        }
    }

    private void setFlywheelRPM(double rpm) {
        double tps = (rpm / 60.0) * TICKS_PER_REV;
        flyWheelLeft.setVelocity(tps);
        flyWheelRight.setVelocity(tps);
    }

    private void setHoodAngle(double deg) {
        deg = Math.max(MIN_HOOD_ANGLE, Math.min(MAX_HOOD_ANGLE, deg));

        double pos = HOOD_MIN_POS +
                (deg / MAX_HOOD_ANGLE) * (HOOD_MAX_POS - HOOD_MIN_POS);

        hoodServo.setPosition(pos);
    }

    public double getCurrentRPM() {
        double l = flyWheelLeft.getVelocity();
        double r = flyWheelRight.getVelocity();
        return ((l + r) / 2) / TICKS_PER_REV * 60;
    }

    public boolean isAtSpeed() {
        if (lastTargetRPM <= 0) return false;
        return Math.abs(getCurrentRPM() - lastTargetRPM) < lastTargetRPM * 0.05;
    }

    // ------------------------ CALIBRATION & TELEMETRY ------------------------

    public static class ShooterSettings {
        public double rpm;
        public double hoodAngleDegrees;

        public ShooterSettings(double r, double h) {
            rpm = r;
            hoodAngleDegrees = h;
        }

        public String toString() {
            return String.format("RPM %.0f | Hood %.1f°", rpm, hoodAngleDegrees);
        }
    }
}
