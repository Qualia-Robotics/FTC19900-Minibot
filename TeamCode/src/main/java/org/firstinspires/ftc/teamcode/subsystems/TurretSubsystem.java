package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TurretSubsystem {
    private final double turretMax = 0.85;
    private final double turretMin = 0.15;
    private final double turnSpeed = 0.8;

    private double TurretPos = 0;



    public enum State {AUTO, IDLE, MANUAL}

    private final CRServo turntableServo, leftVerticalServo, rightVerticalServo;
    private final int horizontalTolerance = 10;
    private final int verticalTolerance = 10;

    private State state = State.IDLE;

    private double targetHorizontalPosition = 0;
    private double targetVerticalPosition = 0;

    private double currentHorizontalPosition = 0;
    private double currentVerticalPosition = 0;



    private boolean busy = false;

    public TurretSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        turntableServo = hardwareMap.get(CRServo.class, "turntableServo");
        leftVerticalServo = hardwareMap.get(CRServo.class, "leftVerticalServo");
        rightVerticalServo = hardwareMap.get(CRServo.class, "rightVerticalServo");

        // Set directions (adjust if movement is inverted) ----------
        turntableServo.setDirection(CRServo.Direction.FORWARD);
        leftVerticalServo.setDirection(CRServo.Direction.REVERSE);
        rightVerticalServo.setDirection(CRServo.Direction.REVERSE);

      }
    // Manual Aiming
    public void manualAiming(double horizontal, double vertical){
        double turnPower = horizontal * turnSpeed;
        turntableServo.setPower(turnPower);
//

    }

    // Look For Game Objects
    public void lookForGameObjects() {
        if (busy) {
            return;
        }
        state = State.AUTO;
        busy = true;


        currentHorizontalPosition = turntableServo.getPower();
        currentVerticalPosition = leftVerticalServo.getPower();


//        targetHorizontalPosition = 0;
//        targetVerticalPosition = 0;
//        if (targetHorizontalPosition > 0){
//            while (targetHorizontalPosition != currentHorizontalPosition) {
//                //null_motor.setPower(0.5);
//            }
//        } else {
//            while (targetHorizontalPosition != currentHorizontalPosition) {
//                //null_motor.setPower(-0.5);
//            }
//        }
//        lefVerticalServo.setPosition(targetVerticalPosition);
    }


    public void addTelemetry (Telemetry telemetry){
        telemetry.addLine("----- Turret -----");
        telemetry.addData("Turret State = ", state);
        telemetry.addData("Turret Horizontal Position = ", currentHorizontalPosition);
        telemetry.addData("Turret Vertical Position = ", currentVerticalPosition);


    }

}

