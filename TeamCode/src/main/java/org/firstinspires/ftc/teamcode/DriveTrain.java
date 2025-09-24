package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class DriveTrain
{


    /* Public OpMode members. */
    private DcMotor  leftFrontDriveWheel   = null; //motor for left front wheel
    private DcMotor  leftBackDriveWheel   = null; //motor for left back wheel
    private DcMotor  rightFrontDriveWheel  = null; //motor for right front wheel
    private DcMotor  rightBackDriveWheel  = null; //motor for right back wheel

    public DriveTrain(HardwareMap hwMap) {
        // Define and Initialize Motors
        leftFrontDriveWheel  = hwMap.get(DcMotor.class, "leftFrontDriveWheel");
        leftBackDriveWheel  = hwMap.get(DcMotor.class, "leftBackDriveWheel");
        rightFrontDriveWheel = hwMap.get(DcMotor.class, "rightFrontDriveWheel");
        rightBackDriveWheel = hwMap.get(DcMotor.class, "rightBackDriveWheel");

        //left wheels are counterclockwise and because of 90 degree gearboxes, they all are forward
        leftFrontDriveWheel.setDirection(DcMotor.Direction.FORWARD);// Set to REVERSE if using AndyMark motors
        leftBackDriveWheel.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDriveWheel.setDirection(DcMotor.Direction.FORWARD);
        rightBackDriveWheel.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors

        // Set all motors to zero power
        leftFrontDriveWheel.setPower(0);
        leftBackDriveWheel.setPower(0);
        rightFrontDriveWheel.setPower(0);
        rightBackDriveWheel.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftFrontDriveWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDriveWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDriveWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDriveWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    }
    //Drives the robot
    //Forward = how fast to move forward
    //Right = how fast to move right
    //Clockwise = how fast to move clockwise
    public void drive(double forward, double right, double clockwise)
    {

        //Controls the speed for going across the field; can be changed if going too slow
        //strafe is lateral movement!!! Looking forward while moving sideways
        //if(gamepad1.right_bumper)
        //{
        //  forward /= 4;
        //   strafe /= 4;
        //   turn /= 4;
        //}

        double denominator = Math.max(Math.abs(forward) + Math.abs(right) + Math.abs(clockwise), 1);

        rightFrontDriveWheel.setPower((forward - right - clockwise) / denominator);
        leftFrontDriveWheel.setPower((forward + right + clockwise) / denominator);
        leftBackDriveWheel.setPower((forward - right +  clockwise) / denominator);
        rightBackDriveWheel.setPower((forward +  right - clockwise) / denominator);
    }
}
