/* Created by Phil Malone. 2023.
    This class illustrates my simplified Odometry Strategy.
    It implements basic straight line motions but with heading and drift controls to limit drift.
    See the readme for a link to a video tutorial explaining the operation and limitations of the code.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This OpMode illustrates an autonomous opmode using simple Odometry
 * All robot functions are performed by an external "Robot" class that manages all hardware interactions.
 * Pure Drive or Strafe motions are maintained using two Odometry Wheels.
 * The IMU gyro is used to stabilize the heading during all motions
 */

@Autonomous(name="Human Autonomous", group = "AUTO")
@Disabled
public class Human_Auto extends LinearOpMode {
    // get an instance of the "Robot" class.
   // private SimplifiedOdometryRobot robot = new SimplifiedOdometryRobot(this);
    private DcMotor Linear_up = null;
    private DcMotor Linear_Forward = null;
    private CRServo Servo_bucket;
    private Servo servo_arm_1;
    private double Servo_bucket_pos = -0.8;
    private boolean intake_pos_toggle = true;
    private double intake_pos_low = 0.15;
    private double intake_pos_high = 0.9;
    private DistanceSensor Linear_Up_Dist = null;
    private ColorSensor Linear_Fwd_Color = null;

    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        // Initialize the robot hardware & Turn on telemetry
    //    robot.initialize(true);

        // Wait for driver to press start
        telemetry.addData(">", "Touch Play to run Auto");
        telemetry.update();

        waitForStart();
      //  robot.resetHeading();  // Reset heading to set a baseline for Auto

        // Run Auto if stop was not pressed.
        if (opModeIsActive()) {
            //Linear_up = hardwareMap.get(DcMotor.class, "Linear_up");
            //Servo_bucket = hardwareMap.get(CRServo.class, "SBucket");
            //servo_arm_1 = hardwareMap.get(Servo.class, "servoarm1");
            //Linear_Up_Dist = hardwareMap.get(DistanceSensor.class, "Linear_Up_Dist");
            //Linear_Forward = hardwareMap.get(DcMotor.class, "Linear_FW");
            //Linear_Fwd_Color = hardwareMap.get(ColorSensor.class, "Linear_Fwd_Color");

            //Linear_Forward.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //Linear_Forward.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //Linear_up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //Linear_up.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //Linear_up.setDirection(DcMotorSimple.Direction.REVERSE);

            //Servo_bucket.setPower(Servo_bucket_pos);
            //servo_arm_1.setPosition(intake_pos_high);

            // Initialize the robot hardware & Turn on telemetry
         //   robot.initialize(true);

            // Wait for driver to press start
            telemetry.addData(">", "Touch Play to run Auto");
            telemetry.update();

            waitForStart();
          //  robot.resetHeading();  // Reset heading to set a baseline for Auto

            // Run Auto if stop was not pressed.
                // Note, this example takes more than 30 seconds to execute, so turn OFF the auto timer.

                // Drive a large rectangle, turning at each corner
//            robot.drive(  12, 0.60, 0.25);
//            robot.turnTo(90, 0.45, 0.5);
//            robot.drive(  2, 0.60, 0.25);
//            robot.turnTo(180, 0.45, 0.5);
//            robot.drive(  12, 0.60, 0.25);
//            robot.turnTo(270, 0.45, 0.5);
//            robot.drive(  72, 0.60, 0.25);
//            robot.turnTo(0, 0.45, 0.5);
//            sleep(500);

                // Drive the path again without turning
                //robot.drive( 4, 0.60, 0.15);
                //robot.strafe( -48, 0.60, 0.15);
                //robot.drive( -4, 0.60, 0.15);

             //   robot.drive(6, 0.20, 0.15);
             //   robot.strafe(-50, 0.20, 0.15);
             //   robot.drive(-8, 0.20, 0.15);

                //Linear_Forward.setPower(0.75);

                /*
            if (Linear_Forward.getCurrentPosition()<-750) {
                Linear_Forward.setPower(0);
            }
                sleep(500);

            if (Linear_up.getCurrentPosition()<16400) {
                Linear_up.setPower(1);
            } else Linear_up.setPower(0);

                sleep(750);

                Servo_bucket.setPower(0.6);
                sleep(750);
                Servo_bucket.setPower(-0.8);
                sleep(750);

                if ((Linear_up.getCurrentPosition()>1  || Linear_Up_Dist.getDistance(DistanceUnit.CM)>10.0)) {
                Linear_up.setPower(-1);
                sleep(2300);
                Linear_up.setPower(0);
                sleep(500);
                Linear_Forward.setPower(0);

                sleep(1900);
                Linear_Forward.setPower(0);


                telemetry.addData("ServoBucket:", Servo_bucket.getPower());

                telemetry.addData("Forward encoder value", Linear_Forward.getCurrentPosition());
                telemetry.addData("Up encoder value", Linear_up.getCurrentPosition());
                */


        }
    }
}