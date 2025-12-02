package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Objects;
public class turretSubsystem {
    public enum State {AUTO, IDLE, MANUAL}

    public final DcMotorEx null_motor;
    public final Servo null_servo, null_servo_2;
    private final int horizontalTolerance = 10;
    private final int verticalTolerance = 10;

    private State state = State.IDLE;

    private double targetHorizontalPosition = 0;
    private double targetVerticalPosition = 0;

    private double currentHorizontalPosition = 0;
    private double currentVerticalPosition = 0;



    private boolean busy = false;

    public turretSubsystem(HardwareMap hardwareMap) {
        null_motor = hardwareMap.get(DcMotorEx.class, "null");
        null_servo = hardwareMap.get(Servo.class, "null_servo");
        null_servo_2 = hardwareMap.get(Servo.class, "null_servo_2");

        // Encoder logic -------------------------------------------------
        null_motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        null_motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Set directions (adjust if movement is inverted) ----------
        null_motor.setDirection(DcMotorEx.Direction.REVERSE);
        null_servo.setDirection(Servo.Direction.REVERSE);
        null_servo_2.setDirection(Servo.Direction.REVERSE);


        // Set motor behavior ----------------------------------------------
        null_motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }
    // Manual Aiming
    public void manualAiming(double targetHorizontalPosition, double targetVerticalPosition){
        null_motor.setPower(targetHorizontalPosition);
        null_servo.setPosition(targetVerticalPosition);
    }

    // Look For Game Objects
    public void lookForGameObjects() {
        if (busy) {
            return;
        }
        state = State.AUTO;
        busy = true;

        null_motor.setTargetPositionTolerance(horizontalTolerance);
        null_motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        currentHorizontalPosition = null_motor.getCurrentPosition();
        currentVerticalPosition = null_servo.getPosition();


        targetHorizontalPosition = 0;
        targetVerticalPosition = 0;
        if (targetHorizontalPosition > 0){
            while (targetHorizontalPosition != currentHorizontalPosition) {
                null_motor.setPower(0.5);
            }
        } else {
            while (targetHorizontalPosition != currentHorizontalPosition) {
                null_motor.setPower(-0.5);
            }
        }
        null_servo.setPosition(targetVerticalPosition);
    }


    public void addTelemetry (Telemetry telemetry){
        telemetry.addLine("----- Turret -----");
        telemetry.addData("Turret State = ", state);
        telemetry.addData("Turret Horizontal Position = ", currentHorizontalPosition);
        telemetry.addData("Turret Vertical Position = ", currentVerticalPosition);


    }

}

