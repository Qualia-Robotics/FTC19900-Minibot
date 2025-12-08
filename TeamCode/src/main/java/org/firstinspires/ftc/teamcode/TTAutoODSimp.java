package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.onbotjava.handlers.admin.ResetOnBotJava;

@Autonomous(name = "TTAutoOD6.java", group = "Fall2025")
@Disabled
public class TTAutoODSimp extends LinearOpMode {
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
    public static final double STRAFE_GROUP_1 = -9.5;

    /** Strafe distance for second ball group (inches) */
    public static final double STRAFE_GROUP_2 = -33.5;

    /** Strafe distance for third ball group (inches) */
    public static final double STRAFE_GROUP_3 = -57.5;

    /** Array of all strafe distances for easy iteration */
    public static final double[] STRAFE_DISTANCES = {
            STRAFE_GROUP_1,
            STRAFE_GROUP_2,
            STRAFE_GROUP_3
    };

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
            // Step 1: Back up to shooting position
            odometry.drive(-42, 1, 0);

            // Step 2: Shoot pre-loaded balls
            shootBalls();

            // Loop through 3 ball groups with different strafe distances
            for (int i = 0; i < STRAFE_DISTANCES.length; i++) {
                double strafeDistance = STRAFE_DISTANCES[i];

                // Step 3: Rotate counterclockwise to collection angle
                rotateRelative(135, 0.6);

                // Step 4: Strafe left to ball group
                odometry.strafe(-strafeDistance, 1, 0);

                // Step 5: Drive forward while collecting balls
                pickupBalls();
                driveAndCollect(-32, 0.5);
                stoppickupBalls();
                // Step 6: Back up to shooting lane
                odometry.drive(32, 1, 0);

                // Step 7: Strafe right (return to shooting position)
                odometry.strafe(strafeDistance, 1, 0);

                // Step 8: Rotate clockwise to shooting angle
                rotateRelative(-135, 0.6);

                // Step 9: Shoot collected balls
                ballDrop();
                shootBalls();
            }

            // Exit autonomous loop
            break;
        }
    }

    // Helper method to shoot balls
    private void ballDrop(){
        index.setPower(0.85);
        sleep(300);
        index.setPower(0);
    }
    private void stoppickupBalls(){
        rotate.setPower(0);
        rotate2.setPower(0);
    }
    private void pickupBalls(){
        rotate.setPower(-1);
        rotate2.setPower(1);
    }
    private void shootBalls() {
        leftlaunch.setPower(0.83);
        rightlaunch.setPower(-0.83);
        sleep(400);
        index.setPower(-0.85);
        rotate.setPower(-1);
        rotate2.setPower(1);
        sleep(2700);

        // Stop all mechanisms
        leftlaunch.setPower(0);
        rightlaunch.setPower(0);
        index.setPower(0);
        rotate.setPower(0);
        rotate2.setPower(0);
    }

    // Helper method to drive forward and collect balls
    private void driveAndCollect(double inches, double power) {
        // Start intake mechanism
        index.setPower(-1);

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
        odometry.turnTo(targetHeading, power, 1);
    }
}