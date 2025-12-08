package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "TTMain.java", group = "LinearOpMode")

public class TTMain extends LinearOpMode {
  //  private CRServo rotate3;
   // private CRServo rotate4;
    private DcMotor leftlaunch;
    private CRServo rotate;
    private CRServo rotate2;
    private DcMotor rightlaunch;
    private DcMotor index;

    private DcMotor backleft;
    private DcMotor backright;
    private DcMotor frontleft;
    private DcMotor frontright;




    @Override
    public void runOpMode() {
        rotate = hardwareMap.get(CRServo.class, "rotate");
        rotate2 = hardwareMap.get(CRServo.class, "rotate2");
       // rotate3 = hardwareMap.get(CRServo.class, "rotate3");
       // rotate4 = hardwareMap.get(CRServo.class, "rotate4");
        leftlaunch = hardwareMap.get(DcMotor.class, "leftlaunch");
        rightlaunch = hardwareMap.get(DcMotor.class, "rightlaunch");
        index = hardwareMap.get(DcMotor.class, "index");
        backleft = hardwareMap.get(DcMotor.class, "backleft");
        backright = hardwareMap.get(DcMotor.class, "backright");
        frontleft = hardwareMap.get(DcMotor.class, "frontleft");
        frontright = hardwareMap.get(DcMotor.class, "frontright");


        frontleft.setDirection(DcMotor.Direction.FORWARD);
        backleft.setDirection(DcMotor.Direction.FORWARD);
        frontright.setDirection(DcMotor.Direction.REVERSE);
        backright.setDirection(DcMotor.Direction.REVERSE);


     telemetry.addData("Status", "Initialized");
     waitForStart();

     telemetry.update();
     while (opModeIsActive()) {
         // Put run blocks here.


         double max;
         // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
         double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
         double lateral = gamepad1.left_stick_x;
         double yaw = gamepad1.right_stick_x;

         // Combine the joystick requests for each axis-motion to determine each wheel's power.
         // Set up a variable for each drive wheel to save the power level for telemetry.
         double frontLeftPower = axial + lateral + yaw;
         double frontRightPower = axial - lateral - yaw;
         double backLeftPower = axial - lateral + yaw;
         double backRightPower = axial + lateral - yaw;

         // Normalize the values so no wheel power exceeds 100%
         // This ensures that the robot maintains the desired motion.
         max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
         max = Math.max(max, Math.abs(backLeftPower));
         max = Math.max(max, Math.abs(backRightPower));
         if (max > 1.0) {
             frontLeftPower /= max;
             frontRightPower /= max;
             backLeftPower /= max;
             backRightPower /= max;
         }
         frontleft.setPower(frontLeftPower);
         frontright.setPower(frontRightPower);
         backleft.setPower(backLeftPower);
         backright.setPower(backRightPower);

         if (gamepad1.right_bumper) {
             frontleft.setPower(frontLeftPower * 0.9);
             frontright.setPower(frontRightPower * 0.9);
             backleft.setPower(backLeftPower * 0.9);
             backright.setPower(backRightPower * 0.9);
         } else {
             frontleft.setPower(frontLeftPower * 0.6);
             frontright.setPower(frontRightPower * 0.6);
             backleft.setPower(backLeftPower * 0.6);
             backright.setPower(backRightPower * 0.6);
         }

         if (gamepad2.a) {
             leftlaunch.setPower(0.8);
             rightlaunch.setPower(-0.8);
         } else if (gamepad2.b) {
             leftlaunch.setPower(0.95);
             rightlaunch.setPower(-0.95);
         } else if (gamepad2.x) {
             leftlaunch.setPower(0.85);
             rightlaunch.setPower(-0.85);
         } else if (gamepad2.y) {
             leftlaunch.setPower(0.9);
             rightlaunch.setPower(-0.9);
         } else {
             leftlaunch.setPower(0);
             rightlaunch.setPower(0);
         }


         if (gamepad2.dpad_left) {
             index.setPower(-1);
             rotate.setPower(1);
             rotate2.setPower(-1);
         } else if (gamepad2.dpad_right) {
             index.setPower(1);

         } else if (gamepad2.dpad_down) {
             rotate.setPower(-1);
             rotate2.setPower(1);

         }  else {
             index.setPower(0);
             rotate.setPower(0);
             rotate2.setPower(0);
            }

        // if (gamepad1.a) {
           //  rotate3.setPower(1);
             //rotate4.setPower(1);
        // }
        // else if (gamepad1.b) {

           //  rotate3.setPower(-1);
           //  rotate4.setPower(-1);
        // }
       //  else {
        //     rotate3.setPower(0);
         //    rotate4.setPower(0);
        // }

         telemetry.update();
     }
    }
}


