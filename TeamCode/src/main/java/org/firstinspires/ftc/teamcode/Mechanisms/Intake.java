package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Intake {
    private DcMotor SIntake;
    private DcMotor NSIntake;
    boolean motorRunning = false;
    boolean lastButtonState = false;
    boolean motorRunning2 = false;
    boolean lastButtonState2 = false;

    public void init(HardwareMap hwMap) {
        NSIntake = hwMap.get(DcMotor.class, "intake1");
        SIntake = hwMap.get(DcMotor.class, "intake2");
        NSIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        NSIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void NonStationary(boolean input1) {

        if (input1 && !lastButtonState) {
            motorRunning = !motorRunning;
        }
        NSIntake.setPower(motorRunning ? 1.0 : 0.0);
        lastButtonState = input1;
    }
    public void stationary(boolean input3) {

        if (input3 && !lastButtonState2) {
            motorRunning2 = !motorRunning2;
        }
        SIntake.setPower(motorRunning2 ? 1 : 0.0);
        lastButtonState2 = input3;
    }
}