package org.firstinspires.ftc.teamcode.mechanisms;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class IntakeMotor {
    private DcMotor intakeMotor;

    private enum IntakeDirection { FORWARD, REVERSE }
    private IntakeDirection intakeDirection;

    private enum IntakeState { ON, OFF, PANIC }
    private IntakeState intakeState;

    private final double MAX_SPEED = 1.0;
    private final double STOP_SPEED = 0.0;
    private double normalPower = 0.7;
    private double currentPower;

    private ElapsedTime timer = new ElapsedTime();
    private double panicTime = 0.10;

    public void init(HardwareMap hwMap, String deviceName) {
        intakeMotor = hwMap.get(DcMotor.class, deviceName);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setZeroPowerBehavior(BRAKE);
        intakeMotor.setPower(STOP_SPEED);

        intakeDirection = IntakeDirection.FORWARD;
        intakeState = IntakeState.OFF;
    }

    public void setForwardMotor() {
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void setReverseMotor() {
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setPower(double pwrNum) {
        normalPower = pwrNum;
    }

    public double getPower() {
        return currentPower;
    }

    public String getDirection() {
        return intakeDirection == IntakeDirection.FORWARD ? "FORWARD" : "REVERSE";
    }

    public String getState() {
        if (intakeState == IntakeState.PANIC)
            return "PANIC";
        else if (intakeState == IntakeState.ON)
            return "ON";
        else
            return "OFF";
    }

    public void run(boolean intakeOn, boolean intakeOff, boolean panic) {
        if (intakeOff) {
            intakeState = IntakeState.OFF;
            intakeMotor.setPower(STOP_SPEED);
        } else {
            switch (intakeState) {
                case OFF:
                    if (intakeOn) {
                        intakeState = IntakeState.ON;
                        intakeDirection = IntakeDirection.FORWARD;
                        intakeMotor.setPower(normalPower);
                    } else if (panic) {
                        timer.reset();
                        intakeState = IntakeState.PANIC;
                        intakeDirection = IntakeDirection.REVERSE;
                        intakeMotor.setPower(-1 * normalPower);
                    }
                    break;
                case ON:
                    if (panic) {
                        timer.reset();
                        intakeState = IntakeState.PANIC;
                        intakeDirection = IntakeDirection.REVERSE;
                        intakeMotor.setPower(-1 * normalPower);
                    } else if (intakeOn) {
                        currentPower = Math.abs(intakeMotor.getPower()) + 0.05;
                        if (currentPower <= MAX_SPEED)
                            intakeMotor.setPower(currentPower);
                    } else {
                        currentPower = normalPower;
                        intakeMotor.setPower(currentPower);
                    }
                    break;
                case PANIC:
                    if (panic)
                        timer.reset();
                    else if (timer.seconds() >= panicTime) {
                        intakeState = IntakeState.ON;
                        intakeDirection = IntakeDirection.FORWARD;
                        intakeMotor.setPower(normalPower);
                    }
                    break;
            }
        }
    }
    public void setPanicTime(double second) {
        panicTime = second;
    }
}
