package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class OurTeleOp extends OpMode {
    private DcMotorEx flywheel;
    private DcMotor feedRoller;
    private DcMotor leftDrive;
    private CRServo agitator;
    private DcMotor rightDrive;

    private float flyWheelVelocity = 1900;
    private static final int bankVelocity = 1300;
    private static final int farVelocity = 1900;
    private static final int maxVelocity = 2200;

    private boolean flyWheelPowered;
    private boolean agitatorPowered;
    private boolean feedRollerPowered;

    public void init() {
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        feedRoller = hardwareMap.get(DcMotor.class, "coreHex");
        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        agitator = hardwareMap.get(CRServo.class, "servo");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");

        // Establishing the direction and mode for the motors
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setDirection(DcMotor.Direction.REVERSE);
        feedRoller.setDirection(DcMotor.Direction.REVERSE);
        leftDrive.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addLine("a to turn on/off the flywheel");
        telemetry.addLine("b to turn on/off the agitator");
        telemetry.addLine("x to turn on/off the feed roller");
        telemetry.addLine("Left bumper for slow speed");
        telemetry.addLine("right bumper for medium speed");

 
        telemetry.update();
    }

    public void loop() {
        basicMovement();
        turnOnMotors();
        flyWheel();
    }

    public void basicMovement() {
        float x;
        float y;

        x = gamepad1.right_stick_x;
        y = -gamepad1.left_stick_y;
        leftDrive.setPower(y - x);
        rightDrive.setPower(y + x);
    }
    public void turnOnMotors() {
        if(gamepad1.aWasPressed()) {
            if(flyWheelPowered) {
                flyWheelPowered = false;
            } else {
                flyWheelPowered = true;
            }
        }
        if(gamepad1.bWasPressed()) {
            if(agitatorPowered) {
                agitatorPowered = false;
                agitator.setPower(0);
            } else {
                agitatorPowered = true;
                agitator.setPower(1);
            }
        }
        if(gamepad1.xWasPressed()) {
            if(feedRollerPowered) {
                feedRollerPowered = false;
                feedRoller.setPower(0);
            } else {
                feedRollerPowered = true;
                feedRoller.setPower(1);
            }
        }
        if(gamepad1.leftBumperWasPressed()) {
            flyWheelVelocity = bankVelocity;
        }
        if (gamepad1.rightBumperWasPressed()) {
            flyWheelVelocity = farVelocity;
        }
    }
    public void flyWheel() {
        if(flyWheelPowered) {
            flywheel.setVelocity(flyWheelVelocity);
        } else {
            flywheel.setVelocity(0);
        }
    }
}
