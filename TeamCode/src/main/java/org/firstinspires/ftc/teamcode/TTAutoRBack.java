package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.SimplifiedOdometryRobotCustom;

@Autonomous(name = "TTAutoRBack.java", group = "Fall2025")
@Disabled
public class TTAutoRBack extends LinearOpMode {
    // private SimplifiedOdometryRobot robot = new SimplifiedOdometryRobot(this);
    private DcMotor frontleft; // Used to control the left front drive wheel
    private DcMotor frontright; // Used to control the right front drive wheel
    private DcMotor backleft; // Used to control the left back drive wheel
    private DcMotor backright; // Used to control the right back drive wheel
    private DcMotor leftlaunch;
    private DcMotor rightlaunch;
    private DcMotor index;
    private CRServo rotate;
    private CRServo rotate2;

    private SimplifiedOdometryRobotCustom odometry;

    @Override
    public void runOpMode() {

        frontleft = hardwareMap.get(DcMotor.class, "frontleft");
        frontright = hardwareMap.get(DcMotor.class, "frontright");
        backleft = hardwareMap.get(DcMotor.class, "backleft");
        backright = hardwareMap.get(DcMotor.class, "backright");
        leftlaunch = hardwareMap.get(DcMotor.class, "leftlaunch");
        rightlaunch = hardwareMap.get(DcMotor.class, "rightlaunch");
        index = hardwareMap.get(DcMotor.class, "index");
        rotate = hardwareMap.get(CRServo.class, "rotate");
        rotate2 = hardwareMap.get(CRServo.class, "rotate2");

        odometry = new SimplifiedOdometryRobotCustom(this, index, rightlaunch);

        backright.setDirection(DcMotorSimple.Direction.FORWARD);
        backleft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontleft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontright.setDirection(DcMotorSimple.Direction.FORWARD);
        waitForStart();

        // Initialize odometry system with starting heading of 45 degrees
        odometry.initialize(true);

        while (opModeIsActive()) {
            odometry.drive(-TTAutoConstantsRBack.DRIVE_TO_SHOOT, TTAutoConstantsRBack.DRIVE_POWER, 0);

        }
    }private void ballDrop(){
        index.setPower(TTAutoConstantsB.INDEX_DROP);
        rotate.setPower(TTAutoConstantsB.ROTATE_SERVO_POWER);
        rotate2.setPower(TTAutoConstantsB.ROTATE2_SERVO_POWER);
        sleep(200);
        index.setPower(0);
    }
    private void stoppickupBalls(){
        rotate.setPower(0);
        rotate2.setPower(0);
    }
    private void pickupBalls(){
        rotate.setPower(TTAutoConstantsB.ROTATE_SERVO_POWER);
        rotate2.setPower(TTAutoConstantsB.ROTATE2_SERVO_POWER);
    }
    private void launcherSpoll(){
        leftlaunch.setPower(TTAutoConstantsB.LEFT_LAUNCH_POWER);
        rightlaunch.setPower(TTAutoConstantsB.RIGHT_LAUNCH_POWER);
    }
    private void shootBalls() {
        index.setPower(TTAutoConstantsB.INDEX_SHOOT_POWER);
        rotate.setPower(TTAutoConstantsB.ROTATE_SERVO_POWER);
        rotate2.setPower(TTAutoConstantsB.ROTATE2_SERVO_POWER);
        sleep(TTAutoConstantsB.SHOOTING_DURATION1);
        // Stop all mechanisms
        leftlaunch.setPower(0);
        rightlaunch.setPower(0);
        index.setPower(0);
        rotate.setPower(0);
        rotate2.setPower(0);
    }
    private void finalshootBalls() {
        index.setPower(TTAutoConstantsB.INDEX_SHOOT_POWER);
        rotate.setPower(TTAutoConstantsB.ROTATE_SERVO_POWER);
        rotate2.setPower(TTAutoConstantsB.ROTATE2_SERVO_POWER);
    }


    // Helper method to drive forward and collect balls
    private void driveAndCollect(double inches, double power) {
        // Start intake mechanism
        index.setPower(TTAutoConstantsB.INDEX_COLLECT_POWER);

        // Drive forward while collecting
        odometry.drive(inches, power, 0);

        // Stop intake
        index.setPower(0);
    }

    // Helper method to rotate relative to current heading
    private void rotateRelative(double degrees, double power) {
        // Get current heading from odometry
        double currentHeading = odometry.getHeading();

        // Calculate target heading
        double targetHeading = currentHeading + degrees;

        // Normalize to 0-360 range
        while (targetHeading >= 360)
            targetHeading -= 360;
        while (targetHeading < 0)
            targetHeading += 360;

        // Turn to target heading
        odometry.turnTo(targetHeading, power, TTAutoConstantsB.ROTATION_TOLERANCE);
    }

}
