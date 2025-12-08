package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Test 2: Rotation Test
 * Tests the 315 degree rotations (both counterclockwise and clockwise)
 */
@Autonomous(name = "Test2: Rotation 315deg", group = "Tests")
@Disabled
public class TTAutoODTest2_Rotation extends LinearOpMode {
    private DcMotor frontleft;
    private DcMotor frontright;
    private DcMotor backleft;
    private DcMotor backright;
    private DcMotor leftlaunch;
    private DcMotor rightlaunch;
    private DcMotor index;
    private CRServo rotate;
    private CRServo rotate2;

    private SimplifiedOdometryRobotCustom odometry;

    @Override
    public void runOpMode() {
        // Hardware initialization
        frontleft = hardwareMap.get(DcMotor.class, "frontleft");
        frontright = hardwareMap.get(DcMotor.class, "frontright");
        backleft = hardwareMap.get(DcMotor.class, "backleft");
        backright = hardwareMap.get(DcMotor.class, "backright");
        leftlaunch = hardwareMap.get(DcMotor.class, "leftlaunch");
        rightlaunch = hardwareMap.get(DcMotor.class, "rightlaunch");
        index = hardwareMap.get(DcMotor.class, "index");
        rotate = hardwareMap.get(CRServo.class, "rotate");
        rotate2 = hardwareMap.get(CRServo.class, "rotate2");

        odometry = new SimplifiedOdometryRobotCustom(this, index,rightlaunch);

        // Set motor directions
        backright.setDirection(DcMotorSimple.Direction.FORWARD);
        backleft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontleft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontright.setDirection(DcMotorSimple.Direction.FORWARD);

        telemetry.addData("Status", "Ready to test rotations");
        telemetry.addData("Test", "CCW " + Math.abs(TTAutoConstants.ROTATE_TO_COLLECT) + "° then CW "
                + TTAutoConstants.ROTATE_TO_SHOOT + "°");
        telemetry.update();

        waitForStart();

        // Initialize odometry at starting heading
        odometry.initialize(true);
        while (opModeIsActive()) {
            double startHeading = odometry.getHeading();
            telemetry.addData("Start Heading", "%.1f degrees", startHeading);
            telemetry.update();
            sleep(2000);

            // Test: Rotate counterclockwise
            telemetry.addData("Status", "Rotating CCW " + Math.abs(TTAutoConstants.ROTATE_TO_COLLECT) + "°...");
            telemetry.update();
            rotateRelative(TTAutoConstants.ROTATE_TO_COLLECT, TTAutoConstants.ROTATE_POWER);
            double afterCCW = odometry.getHeading();
            telemetry.addData("After CCW", "%.1f degrees", afterCCW);
            telemetry.update();
            sleep(3000);

            // Test: Rotate clockwise (should return close to start)
            telemetry.addData("Status", "Rotating CW " + TTAutoConstants.ROTATE_TO_SHOOT + "°...");
            telemetry.update();
            rotateRelative(TTAutoConstants.ROTATE_TO_SHOOT, TTAutoConstants.ROTATE_POWER);
            double finalHeading = odometry.getHeading();
            telemetry.addData("Final Heading", "%.1f degrees", finalHeading);
            telemetry.addData("Drift", "%.1f degrees", Math.abs(finalHeading - startHeading));
            telemetry.update();

            sleep(10000);
            break;
        }
    }

    private void rotateRelative(double degrees, double power) {
        double currentHeading = odometry.getHeading();
        double targetHeading = currentHeading + degrees;

        while (targetHeading >= 360)
            targetHeading -= 360;
        while (targetHeading < 0)
            targetHeading += 360;

        odometry.turnTo(targetHeading, power, TTAutoConstants.ROTATION_TOLERANCE);
    }
}
