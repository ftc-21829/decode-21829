package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.core.command.utility.InstantCommand;
import com.rowanmcalpin.nextftc.core.command.utility.delays.Delay;
import com.rowanmcalpin.nextftc.core.command.utility.delays.WaitUntil;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.testing.RTPAxon;

@Config
public class AllMechCopy {
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
    private RTPAxon axon;
    VoltageSensor battery;



    // ========== TURRET PID CONSTANTS ==========
    public static double turret_kP = 0.005, turret_kI = 0.0005, turret_kD = 0.004;
    public static double turret_tolerance = 1.0;
    public static double turret_max_power = 1;
    public static double turret_deadband = 0.25;
    private double integral = 0.0;
    private double lastError = 0.0;

    // ========== FIELD-RELATIVE TRACKING CONSTANTS ==========
    // Target position on field (in inches) - ADJUST FOR YOUR TARGET
    public static double TARGET_X = 0; // X coordinate of basket/target
    public static double TARGET_Y = 144; // Y coordinate of basket/target

    // Turret offset from robot center (radians) - adjust if turret isn't centered
    public static double TURRET_OFFSET = 0.0;

    // Whether to use field-relative (true) or vision-based (false) tracking
    public static boolean USE_FIELD_RELATIVE_TRACKING = true;

    // AprilTag pose fusion weight (0.0 = trust odometry only, 1.0 = trust AprilTag only)
    public static double APRILTAG_FUSION_WEIGHT = 0.2;

    // ========== SHOOTER PID ==========
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

    // ========== SERVO POSITIONS ==========
    public static double door_open_pos = 0.75;
    public static double door_close_pos = 0.6;
    public static double butt_kicker_down = 0.88;
    public static double butt_kicker_up = 0.4675;

    // ========== SHOOTER TUNING ==========
    private final double HOOD_A = 0.234833;
    private final double HOOD_B = 1.00578;
    private final double OUTTAKE_A = 0.39845;
    private final double OUTTAKE_B = 1.0043;

    // ========== SHOOTER STATE ==========
    private double lastValidDistanceMM = -1.0;
    private double targetOuttakePower = 0.0;
    private boolean shooterEnabled = false;

    // ========== RISING EDGE DETECTORS ==========
    private boolean prevDpadUp = false;
    private boolean prevDpadDown = false;
    private boolean prevLogButton = false;
    public static double kps = 0.00515;
    public static double kis = 0;
    public static double kds = 0.000045;

    public AllMechCopy(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.telemetry = new MultipleTelemetry();

        follower = Constants.createFollower(hardwareMap);
        AnalogInput encoder = hardwareMap.get(AnalogInput.class, "turretAnalog");
        turretServo = hardwareMap.get(CRServo.class, "turret");

        axon = new RTPAxon(turretServo, encoder);



        axon.setMaxPower(1);
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        outtakeLow = hardwareMap.get(DcMotorEx.class, "outtake Low");
        outtakeHigh = hardwareMap.get(DcMotorEx.class, "outtake High");
        outtakeHigh.setDirection(DcMotorSimple.Direction.REVERSE);
        battery = hardwareMap.voltageSensor.iterator().next();

        outtakeHigh.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeLow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        outtakeHigh.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeHigh.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outtakeLow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeLow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        follower.setStartingPose(new Pose(72, 72, Math.toRadians(90)));


        intake = hardwareMap.get(DcMotorEx.class, "intake");
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");


        door = hardwareMap.get(Servo.class, "door");
        door.setPosition(door_open_pos);
        hood = hardwareMap.get(Servo.class, "hood");
        buttkicker = hardwareMap.get(Servo.class, "buttkicker");
        buttkicker.setPosition(butt_kicker_down);

        initializeLimelight(hardwareMap);

        lastLowPosPID = outtakeLow.getCurrentPosition();
        lastHighPosPID = outtakeHigh.getCurrentPosition();
        lastTimePID = System.nanoTime();
        axon.setPidCoeffs(kps, kis, kds);

    }

    private void initializeLimelight(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(0);
    }

    public void updateLimelightData() {
        currentResult = limelight.getLatestResult();

        // Optional: Use AprilTag detections for pose correction
        if (USE_FIELD_RELATIVE_TRACKING && currentResult != null && currentResult.isValid()) {
            updatePoseFromAprilTag();
        }
    }

    private void updatePoseFromAprilTag() {
        // TODO: Implement AprilTag pose fusion for continuous pose correction
        // This would involve:
        // 1. Getting the AprilTag's known field position
        // 2. Using botpose or camera-to-target transform to calculate robot pose
        // 3. Fusing with Pinpoint odometry using APRILTAG_FUSION_WEIGHT

        // Example skeleton:
        // Pose aprilTagPose = getAprilTagPoseFromLimelight(currentResult);
        // Pose odometryPose = follower.getPose();
        // Pose fusedPose = blendPoses(odometryPose, aprilTagPose, APRILTAG_FUSION_WEIGHT);
        // follower.setPose(fusedPose);
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
            axon.setRtp(false);
            axon.setPower(0.0);
            integral = 0.0;
            lastError = 0.0;
        }
    }

    public boolean FieldRelativeTrue() {
        return USE_FIELD_RELATIVE_TRACKING;
    }


    public boolean isTurretTrackingActive() {
        return turretTrackingActive;
    }

    // ========== FIELD-RELATIVE TURRET TRACKING ==========

    /**
     * Calculate the turret error angle using field-relative positioning from Pinpoint odometry.
     * Returns the angle error in degrees (positive = turn right, negative = turn left).
     */
    private double calculateFieldRelativeTurretError() {
        // Get current robot pose from Pinpoint via Pedro Pathing
        Pose robotPose = follower.getPose();

        double robotX = robotPose.getX();
        double robotY = robotPose.getY();
        double robotHeading = robotPose.getHeading(); // In radians

        // Calculate vector from robot to target
        double dx = TARGET_X - robotX;
        double dy = TARGET_Y - robotY;

        // Calculate absolute angle to target
        double angleToTarget = Math.atan2(dy, dx);

        // Calculate turret angle relative to robot
        double turretAngle = angleToTarget - robotHeading;

        // Normalize to [-π, π]
        turretAngle = Math.atan2(Math.sin(turretAngle), Math.cos(turretAngle));

        // Convert to degrees for error (to match Limelight tx units)
        return Math.toDegrees(turretAngle) *2.3793;
    }

    /**
     * Get the distance from robot to target using Pinpoint odometry.
     * Returns distance in inches.
     */

    /**
     * Main turret tracking update loop.
     * Supports both field-relative (Pinpoint) and vision-based (Limelight) tracking.
     */


    public void updateTurretTracking() {
        if (!turretTrackingActive) {
            axon.setPower(0.0);
            return;
        }

        // ================= FIELD RELATIVE =================
        if (USE_FIELD_RELATIVE_TRACKING) {

            // Field-relative uses RTP
            axon.setRtp(true);

            double error = calculateFieldRelativeTurretError();
            axon.update(); // <--- THIS LINE IS MISSING


            // Deadband


            // Target rotation = error in degrees
            axon.setTargetRotation(error);
            return;
        }

        // ================= LIMELIGHT =================
        // Vision-based uses OPEN LOOP POWER
        axon.setRtp(false);

        if (!hasValidTarget()) {
            axon.setPower(0.0);
            return;
        }

        double error = currentResult.getTx();

        // Deadband
        if (Math.abs(error) < turret_deadband) {
            axon.setPower(0.0);
            integral = 0.0;
            lastError = 0.0;
            return;
        }

        // PID → power
        integral += error;
        integral = Math.max(-100.0, Math.min(100.0, integral));
        double derivative = error - lastError;
        double output = (turret_kP * error) + (turret_kI * integral) + (turret_kD * derivative);
        output = -output;
        lastError = error;

        output = Math.max(-turret_max_power, Math.min(turret_max_power, output));
        axon.setPower(output);
    }
//    public void updateTurretTracking() {
//        if (!turretTrackingActive) {
//            axon.setPower(0.0);
//            return;
//        }
//
//        // ================= FIELD RELATIVE =================
//        if (USE_FIELD_RELATIVE_TRACKING) {
//            axon.setRtp(true);
//
//            double errorDegrees = calculateFieldRelativeTurretError();
//
//            // Get current tracking error from RTP
//            double currentError = axon.getTargetRotation() - axon.getTotalRotation();
//
//            // Calculate how much to adjust target (smooth tracking)
//            double adjustment = errorDegrees - currentError;
//
//            // Only adjust if significant change
//            if (Math.abs(adjustment) > 0.5) {
//                axon.changeTargetRotation(adjustment);
//            }
//
//            axon.update();
//
//            telemetry.addData("Field Error", "%.2f°", errorDegrees);
//            telemetry.addData("RTP Error", "%.2f°", currentError);
//            telemetry.addData("Total Rotation", "%.2f°", axon.getTotalRotation());
//
//            return;
//        }
//
//        // ... rest of code
//
//
//        // ================= LIMELIGHT =================
//        axon.setRtp(false);
//
//        if (!hasValidTarget()) {
//            axon.setPower(0.0);
//            return;
//        }
//
//        double error = currentResult.getTx();
//
//        if (Math.abs(error) < turret_deadband) {
//            axon.setPower(0.0);
//            integral = 0.0;
//            lastError = 0.0;
//            return;
//        }
//
//        integral += error;
//        integral = Math.max(-100.0, Math.min(100.0, integral));
//        double derivative = error - lastError;
//        double output = (turret_kP * error) + (turret_kI * integral) + (turret_kD * derivative);
//        output = -output;
//        lastError = error;
//
//        output = Math.max(-turret_max_power, Math.min(turret_max_power, output));
//        axon.setPower(output);
//    }


    public boolean isTurretAligned() {

        return hasValidTarget() && Math.abs(currentResult.getTx()) < turret_tolerance;

    }

    // ========== COMMANDS ==========

    public Command intakeOn() {return new InstantCommand(()-> intake.setPower(1));}
    public Command intakeOff() { return new InstantCommand(() -> intake.setPower(0)); }
    public Command doorOpen() { return new InstantCommand(() -> door.setPosition(door_open_pos)); }
    public Command doorClose() { return new InstantCommand(() -> door.setPosition(door_close_pos)); }
    public Command transferOn() { return new InstantCommand(() -> transfer.setPower(0.75)); }
    public Command transferOff() { return new InstantCommand(() -> transfer.setPower(0)); }
    public Command transferfull() {return new InstantCommand(()-> transfer.setPower(1));}
    public Command transferSlow() { return new InstantCommand(() -> transfer.setPower(0.75)); }

    // ---- OUTTAKE now uses PID flywheel + hood ----
    public Command OuttakeOne(){
        return new SequentialGroup(
                new ParallelGroup(
                        transferfull(),
                        doorOpen(),
                        intakeOn()
                ),

                new Delay(1),
                ButtKicker(),
                new ParallelGroup(
                        OuttakeOff(),
                        intakeOff(),
                        transferOff(),
                        doorClose()
                )
        );
    }
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
    public Command turretOn() {
        return new ParallelGroup(
                new InstantCommand(() -> setTurretTrackingActive(!isTurretTrackingActive())));
    }
    public Command turret() {
        return new InstantCommand(() -> {
            setTurretTrackingActive(false);
            USE_FIELD_RELATIVE_TRACKING = !USE_FIELD_RELATIVE_TRACKING;
            setTurretTrackingActive(true);
        });
    }
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
    // ========== SHOOTER HELPERS ==========

    public double computeHoodPositionFromDistance(double distanceMM) {
        if (distanceMM <= 0) return 0.0;
        return -0.0000455193 * Math.pow(distanceMM, 2)
                + 0.0198468 * distanceMM
                - 1.24468;
    }

    public double computeShooterTargetVelocityFromDistance(double distanceMM) {
        if (distanceMM <= 0) return 0.0;
        shooterTargetVelocity = 0.0426503 * Math.pow(distanceMM, 2)
                - 8.80296 * distanceMM
                + 1675.60075;
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