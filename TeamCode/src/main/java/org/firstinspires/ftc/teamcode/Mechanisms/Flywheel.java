package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;



public class Flywheel {
    private static DcMotorEx FlywheelLMotor,FlywheelRMotor;
    boolean motorRunning = false;
    boolean lastButtonState = false;

    public static void init(HardwareMap hwMap) {
        FlywheelLMotor = hwMap.get(DcMotorEx.class, "flywheelL");
        FlywheelRMotor = hwMap.get(DcMotorEx.class,"flywheelR");
        FlywheelLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FlywheelRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FlywheelRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FlywheelLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FlywheelRMotor.setDirection(DcMotorSimple.Direction.REVERSE);

    }
    public void doubleA(double flywheelPower,boolean input1) {

        if (input1 && !lastButtonState) {
            motorRunning = !motorRunning;
        }
        FlywheelLMotor.setPower(motorRunning ? flywheelPower : 0.0);
        FlywheelRMotor.setPower(motorRunning ? flywheelPower : 0.0);
        lastButtonState = input1;
        }
}