package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
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

import java.io.SequenceInputStream;

@Config
public class AllMechCopy {
    public DcMotorEx intake, outtakeLow, outtakeHigh, transfer;
    public Follower follower;
    public ColorSensor colorSensor, colorSensorFront, colorSensorBack;

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

    com.pedropathing.util.Timer actionTimer;

    // ========== TURRET PID CONSTANTS ==========
    public static double turret_kP = 0.008, turret_kI = 0.0005, turret_kD = 0.004;
    public static double turret_tolerance = 1.0;
    public static double turret_max_power = 1;
    public static double turret_deadband = 0.1;
    private double integral = 0.0;
    private double lastError = 0.0;
    // ========== SHOOTER VELOCITY CONTROL (FF + P) ==========
    public static double shooter_kV = 0.0006;
    public static double shooter_kS = 0.025;
    public static double shooter_kP_vel = 0.0015;


    // ========== FIELD-RELATIVE TRACKING CONSTANTS ==========
    // Target position on field (in inches) - ADJUST FOR YOUR TARGET
    public static double TARGET_X = 4; // X coordinate of basket/target
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
    // ---- Hood shot compensation ----
    public static double SHOT_VEL_DROP = 130;          // encoder ticks/sec, tune
    public static double HOOD_COMP_STEP = 0.2;       // servo units per shot
    public static double HOOD_COMP_MAX = 0.4;        // max hood drop
    public static long SHOT_COOLDOWN_NS = 300_000_000; // 120 ms

    private double hoodComp = 0.0;
    private double lastShooterVel = 0.0;
    private long lastShotTime = 0;


    public double shooterTargetVelocity = 0;
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
    public static double kps = 0.0053; //0.00515
    public static double kis = 0.0001; //0
    public static double kds = 0.0000007; //0.0000005

    public AllMechCopy(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Follower follower) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.telemetry = new MultipleTelemetry();
        this.follower = follower;
        AnalogInput encoder = hardwareMap.get(AnalogInput.class, "turretAnalog");
        turretServo = hardwareMap.get(CRServo.class, "turret");

        axon = new RTPAxon(turretServo, encoder);



        axon.setMaxPower(0.85);
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        colorSensorFront = hardwareMap.get(ColorSensor.class, "colorSensorFront");
        colorSensorBack = hardwareMap.get(ColorSensor.class, "colorSensorBack");

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

        actionTimer = new Timer();
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
        limelight.setPollRateHz(50);
        limelight.start();
        limelight.pipelineSwitch(0);
    }
    public void UpdateTarget(double x, double y){
        TARGET_X = x;
        TARGET_Y = y;
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
    private void onShotDetected() {
        hoodComp = Math.min(hoodComp + HOOD_COMP_STEP, HOOD_COMP_MAX);
        lastShotTime = System.nanoTime();
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
        return Math.toDegrees(turretAngle) * 2.37931024483;
    }

    private double distance(double x, double y){
        return Math.pow((Math.pow((TARGET_X-x),2)+Math.pow((TARGET_Y-y),2)), 0.5);

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


            // Deadband


            // Target rotation = error in degrees
            axon.setTargetRotation(error);
            axon.update(); // <--- THIS LINE IS MISSING

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
    public Command transferfull() {
        return new InstantCommand(() -> transfer.setPower(1));

    }

    public Command transferReverse() {

        return new SequentialGroup(
                new InstantCommand(() -> transfer.setPower(-0.21)),
                new Delay(1),
                transferOff()
        );

    }

    public Command transferSlow() { return new InstantCommand(() -> transfer.setPower(0.75)); }

    // ---- OUTTAKE now uses PID flywheel + hood ----
    public Command OuttakeOne(){
        return new SequentialGroup(
                new ParallelGroup(
                        transferfull(),
                        doorOpen(),
                        intakeOn()
                ),

                new WaitUntil(()->{
                    double distanceCmFront = 100;
                    double distanceCmBack = 100;
                    if(colorSensorFront instanceof DistanceSensor){
                        distanceCmFront = ((DistanceSensor) colorSensorFront).getDistance(DistanceUnit.CM);}
                    if(colorSensorBack instanceof DistanceSensor){
                        distanceCmBack = ((DistanceSensor) colorSensorBack).getDistance(DistanceUnit.CM);}

                    return distanceCmFront > 6.5 && distanceCmBack > 6.5;
                }).then(
                        new SequentialGroup(
                                new Delay(0.3),
                                ButtKicker()

                        )

                        ),
                new ParallelGroup(
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

                        new ParallelGroup(
                                doorClose(),
                                transferReverse()
                        )

        );
    }
    //dse
    public Command intakeAndTransfer() {
        return new ParallelGroup(
                transferOn(),
                intakeOn(),
                transferCheck(),
                doorOpen(),
                new WaitUntil(()->{
                    double distanceCmFront = 100;
                    double distanceCmBack = 100;
                    if(colorSensorFront instanceof DistanceSensor){
                        distanceCmFront = ((DistanceSensor) colorSensorFront).getDistance(DistanceUnit.CM);}
                    if(colorSensorBack instanceof DistanceSensor){
                        distanceCmBack = ((DistanceSensor) colorSensorBack).getDistance(DistanceUnit.CM);}

                    return distanceCmFront < 6.25 && distanceCmBack < 6.25;
                }).then(
                        new ParallelGroup(
                                intakeOff(),
                                transferOff()
                        )

                )
        );
    }



    // ========== SHOOTER HELPERS ==========

    public double computeHoodPositionFromDistance(double distanceMM) {
        if (distanceMM <= 0) return 0.0;
        return 1.75239* Math.pow(10,-8) * Math.pow(distanceMM, 4)
                - 0.00000912987 * Math.pow(distanceMM,3)
                + 0.00166471*Math.pow(distanceMM,2)
                - 0.119039 * distanceMM
                + 2.9607;
    }

    public double computeShooterTargetVelocityFromDistance(double distanceMM) {
        if (distanceMM <= 0) return 0.0;
        shooterTargetVelocity = -0.0000272952 * Math.pow(distanceMM, 4)
                + 0.0113041 * Math.pow(distanceMM,3)
                - 1.65601 * Math.pow(distanceMM, 2)
                + 107.79703 * distanceMM
                - 1542.98727;
        return shooterTargetVelocity;
    }

    public void updateShooterTargetFromField() {
        Pose robotPose = follower.getPose();

        double dmm = distance(robotPose.getX(), robotPose.getY());
            if (dmm > 0) {
                lastValidDistanceMM = dmm;
                targetHoodPosition = computeHoodPositionFromDistance(dmm);
                targetOuttakePower = computeShooterTargetVelocityFromDistance(dmm);
                shooterEnabled = true;
            }

    }
    private double shooterFeedforward(double targetVel) {
        if (Math.abs(targetVel) < 1e-6) return 0.0;
        return shooter_kS * Math.signum(targetVel) + shooter_kV * targetVel;
    }

    private double shooterFeedback(double targetVel, double currentVel) {
        return shooter_kP_vel * (targetVel - currentVel);
    }

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }


    /** Apply PID-controlled flywheel velocity and hood position */
    public void periodicShooterUpdateAndApplyPID() {
        updateShooterTargetFromField();
        if (!shooterEnabled) return;

        int lowPos = outtakeLow.getCurrentPosition();
        int highPos = outtakeHigh.getCurrentPosition();
        long now = System.nanoTime();
        double dt = (now - lastTimePID) / 1e9;

        if (dt <= 0) return;

        double lowVel = (lowPos - lastLowPosPID) / dt;
        double highVel = (highPos - lastHighPosPID) / dt;
        double currentVel = (lowVel + highVel) / 2.0;
        // ---- Shot detection (velocity drop) ----
        long nowTime = System.nanoTime();
        if (lastShooterVel - currentVel > SHOT_VEL_DROP &&
                nowTime - lastShotTime > SHOT_COOLDOWN_NS) {

            onShotDetected();
        }
        lastShooterVel = currentVel;


        lastLowPosPID = lowPos;
        lastHighPosPID = highPos;
        lastTimePID = now;

        // ---- FEEDFORWARD + P CONTROL ----
        double ff = shooterFeedforward(shooterTargetVelocity);
        double fb = shooterFeedback(shooterTargetVelocity, currentVel);
        // ---- Hood recovery when velocity stabilizes ----
        double velError = shooterTargetVelocity - currentVel;
        if (Math.abs(velError) < 150) {
            hoodComp *= 0.85;
            if (hoodComp < 0.06) hoodComp = 0;
        }

        double power = ff + fb;
        power = clamp(power, 0.0, 0.9);

        outtakeLow.setPower(power);
        outtakeHigh.setPower(power);
        hood.setPosition(targetHoodPosition);
    }

}