package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.R;

public class DoubleIntake {

    public DcMotorEx intakeFront, intakeBack;
    public ColorSensor colorSensorFront, colorSensorBack;
    public Gamepad gamepad1;
    public Gamepad gamepad2;
    public Telemetry telemetry;
    public Transfer transfer;
    public IntakeState intakeState;
    private final double INTAKE_POWER = 1;
    private final double CURRENT_THRESHOLD = 1;
    //change with the actual values that we got in that one video
    private final int COLOR_R_MIN_PURPLE = 35;
    private final int COLOR_B_MIN_PURPLE = 60;
    private final int COLOR_G_MIN_PURPLE = 70;
    private final int COLOR_R_MIN_RED = 35;
    private final int COLOR_B_MIN_RED = 60;
    private final int COLOR_G_MIN_RED = 70;
    public Timer ballTimerFrontPurple;
    public Timer ballTimerBackPurple;
    public Timer ballTimerFrontGreen;
    public Timer ballTimerBackGreen;


    public enum IntakeState {
        IDLE,
        INTAKING,
        DETECTED,
        EJECTING

    }

    public DoubleIntake(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        intakeBack = hardwareMap.get(DcMotorEx.class, "intakeBack");
        intakeFront = hardwareMap.get(DcMotorEx.class, "intakeFront");

        colorSensorBack = hardwareMap.get(ColorSensor.class, "colorSensorBack");
        colorSensorFront = hardwareMap.get(ColorSensor.class, "colorSensorFront");

        setState(IntakeState.IDLE);
    }

    public void periodic(){

        switch(intakeState){

            case IDLE:
                intakeBack.setPower(0);
                intakeFront.setPower(0);

            case INTAKING:
                double currentBack = intakeBack.getCurrent(CurrentUnit.MILLIAMPS);
                double currentFront = intakeFront.getCurrent(CurrentUnit.MILLIAMPS);

                int redFront = colorSensorFront.red();
                int blueFront = colorSensorFront.blue();
                int greenFront = colorSensorFront.green();

                int redBack = colorSensorBack.red();
                int blueBack = colorSensorBack.blue();
                int greenBack = colorSensorBack.green();

                if(currentFront > CURRENT_THRESHOLD || currentBack > CURRENT_THRESHOLD){
                    intakeFront.setPower(1);
                    intakeBack.setPower(1);

                }

                if (redFront > 130 && redFront < 160 && blueFront > 180 && blueFront < 235 && greenFront > 1 && greenFront < 75) {
                    gamepad1.rumbleBlips(1);
                    gamepad1.setLedColor(140, 70, 190,2500);
                    ballTimerFrontPurple.resetTimer();

                }
                if (redBack > 130 && redBack < 160 && blueBack > 180 && blueBack < 235 && greenBack > 1 && greenBack < 75) {
                    gamepad2.rumbleBlips(1);
                    gamepad2.setLedColor(140, 70, 190,2500);
                    ballTimerBackPurple.resetTimer();
                }
                if (redFront > 30 && redFront < 80 && blueFront > 110 && blueFront < 180 && greenFront > 155 && greenFront < 215) {
                    gamepad1.rumbleBlips(1);
                    gamepad1.setLedColor(60, 190, 150,2500);
                    ballTimerFrontGreen.resetTimer();
                }
                if (redBack > 30 && redBack < 80 && blueBack > 110 && blueBack < 180 && greenBack > 155 && greenBack < 215) {
                    gamepad1.rumbleBlips(1);
                    gamepad1.setLedColor(60, 190, 150,2500);
                    ballTimerBackGreen.resetTimer();
                }

            case DETECTED:



            case EJECTING:
                intakeBack.setDirection(DcMotorSimple.Direction.REVERSE);
                intakeBack.setPower(1);
                intakeFront.setDirection(DcMotorSimple.Direction.REVERSE);
                intakeFront.setPower(1);

        }

    }






    public void setState(IntakeState newState){
        newState = intakeState;
    }

















}
