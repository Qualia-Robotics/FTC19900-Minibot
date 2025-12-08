package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Test 4: Single Collection/Shoot Cycle
 * Tests one complete collection and shooting cycle (first 12-inch strafe only)
 */
@Autonomous(name = "Test4: Single Cycle", group = "Tests")
@Disabled
public class TTAutoODTest4_SingleCycle extends LinearOpMode {
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

        odometry = new SimplifiedOdometryRobotCustom(this, index, rightlaunch);

        // Set motor directions
        backright.setDirection(DcMotorSimple.Direction.FORWARD);
        backleft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontleft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontright.setDirection(DcMotorSimple.Direction.FORWARD);

        telemetry.addData("Status", "Ready for single cycle test");
        telemetry.addData("Test", "Backup, shoot, collect, return, shoot");
        telemetry.update();

        waitForStart();

        // Initialize odometry at starting heading
        odometry.initialize(true);

        while (opModeIsActive()) {
            // Step 1: Back up to shooting position
            telemetry.addData("Step", "1: Backup " + TTAutoConstants.BACKUP_TO_SHOOT + " inches");
            telemetry.update();
            odometry.drive(-TTAutoConstants.BACKUP_TO_SHOOT, TTAutoConstants.DRIVE_POWER, 0); // Step 2: Shoot
                                                                                              // pre-loaded balls
            telemetry.addData("Step", "2: Shooting pre-loaded balls");
            telemetry.update();
            shootBalls();

            // Step 3: Rotate counterclockwise
            telemetry.addData("Step", "3: Rotate CCW " + Math.abs(TTAutoConstants.ROTATE_TO_COLLECT) + "°");
            telemetry.update();
            rotateRelative(TTAutoConstants.ROTATE_TO_COLLECT, TTAutoConstants.ROTATE_POWER);

            // Step 4: Strafe left
            telemetry.addData("Step", "4: Strafe left " + TTAutoConstants.STRAFE_GROUP_1 + " inches");
            telemetry.update();
            odometry.strafe(-TTAutoConstants.STRAFE_GROUP_1, TTAutoConstants.DRIVE_POWER, 0);

            // Step 5: Drive forward and collect
            telemetry.addData("Step", "5: Drive forward " + TTAutoConstants.DRIVE_TO_COLLECT + "in, collecting");
            telemetry.update();
            driveAndCollect(TTAutoConstants.DRIVE_TO_COLLECT, TTAutoConstants.DRIVE_POWER);

            // Step 6: Back up
            telemetry.addData("Step", "6: Backup " + TTAutoConstants.DRIVE_TO_COLLECT + " inches");
            telemetry.update();
            odometry.drive(-TTAutoConstants.DRIVE_TO_COLLECT, TTAutoConstants.DRIVE_POWER, 0);

            // Step 7: Strafe right
            telemetry.addData("Step", "7: Strafe right " + TTAutoConstants.STRAFE_GROUP_1 + " inches");
            telemetry.update();
            odometry.strafe(TTAutoConstants.STRAFE_GROUP_1, TTAutoConstants.DRIVE_POWER, 0);

            // Step 8: Rotate clockwise
            telemetry.addData("Step", "8: Rotate CW " + TTAutoConstants.ROTATE_TO_SHOOT + "°");
            telemetry.update();
            rotateRelative(TTAutoConstants.ROTATE_TO_SHOOT, TTAutoConstants.ROTATE_POWER); // Step 9: Shoot collected
                                                                                           // balls
            telemetry.addData("Step", "9: Shooting collected balls");
            telemetry.update();
            shootBalls();

            telemetry.addData("Status", "Single cycle complete!");
            telemetry.addData("Final Heading", "%.1f degrees", odometry.getHeading());
            telemetry.update();

            sleep(10000);
            break;
        }
    }

    private void shootBalls() {
        leftlaunch.setPower(TTAutoConstants.LEFT_LAUNCH_POWER);
        rightlaunch.setPower(TTAutoConstants.RIGHT_LAUNCH_POWER);
        sleep(TTAutoConstants.LAUNCHER_SPINUP_TIME);
        index.setPower(TTAutoConstants.INDEX_SHOOT_POWER);
        rotate.setPower(TTAutoConstants.ROTATE_SERVO_POWER);
        rotate2.setPower(TTAutoConstants.ROTATE2_SERVO_POWER);
        sleep(TTAutoConstants.SHOOTING_DURATION1);
        index.setPower(TTAutoConstants.INDEX_SHOOT_POWER2);
        sleep(TTAutoConstants.SHOOTING_DURATION2);
        // Stop all mechanisms
        leftlaunch.setPower(0);
        rightlaunch.setPower(0);
        index.setPower(0);
        rotate.setPower(0);
        rotate2.setPower(0);
    }

    private void driveAndCollect(double inches, double power) {
        index.setPower(TTAutoConstants.INDEX_COLLECT_POWER);
        odometry.drive(inches, power, 0);
        index.setPower(0);
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
