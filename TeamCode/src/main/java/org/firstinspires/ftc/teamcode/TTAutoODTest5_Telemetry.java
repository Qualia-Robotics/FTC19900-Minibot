package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Test 5: Full Autonomous with Detailed Telemetry
 * Runs the complete autonomous routine with detailed telemetry output for
 * monitoring
 */
@Autonomous(name = "Test5: Full Auto + Telemetry", group = "Tests")
@Disabled
public class TTAutoODTest5_Telemetry extends LinearOpMode {
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
    private int cycleCount = 0;

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

        telemetry.addData("Status", "Ready - Full Auto with Telemetry");
        telemetry.addData("Cycles", "4 total (1 preload + 3 collections)");
        telemetry.update();

        waitForStart();

        // Initialize odometry at starting heading
        odometry.initialize(true);

        while (opModeIsActive()) {
            // Initial backup and shoot
            updateTelemetry("Initial", "Backing up " + TTAutoConstants.BACKUP_TO_SHOOT + " inches");
            odometry.drive(-TTAutoConstants.BACKUP_TO_SHOOT, TTAutoConstants.DRIVE_POWER, 0);
            updateTelemetry("Initial", "Shooting pre-loaded balls");
            shootBalls();

            // Loop through 3 ball groups
            double[] strafeDistances = TTAutoConstants.STRAFE_DISTANCES;

            for (int i = 0; i < strafeDistances.length; i++) {
                cycleCount = i + 1;
                double strafeDistance = strafeDistances[i];

                updateTelemetry("Cycle " + cycleCount,
                        "Rotating CCW " + Math.abs(TTAutoConstants.ROTATE_TO_COLLECT) + "°");
                rotateRelative(TTAutoConstants.ROTATE_TO_COLLECT, TTAutoConstants.ROTATE_POWER);
                updateTelemetry("Cycle " + cycleCount, "Strafing left " + (int) strafeDistance + " inches");
                odometry.strafe(-strafeDistance, TTAutoConstants.DRIVE_POWER, 0);

                updateTelemetry("Cycle " + cycleCount,
                        "Collecting balls (" + TTAutoConstants.DRIVE_TO_COLLECT + " inches)");
                driveAndCollect(TTAutoConstants.DRIVE_TO_COLLECT, TTAutoConstants.DRIVE_POWER);

                updateTelemetry("Cycle " + cycleCount, "Returning to lane");
                odometry.drive(-TTAutoConstants.DRIVE_TO_COLLECT, TTAutoConstants.DRIVE_POWER, 0);

                updateTelemetry("Cycle " + cycleCount, "Strafing right " + (int) strafeDistance + " inches");
                odometry.strafe(strafeDistance, TTAutoConstants.DRIVE_POWER, 0);

                updateTelemetry("Cycle " + cycleCount, "Rotating CW " + TTAutoConstants.ROTATE_TO_SHOOT + "°");
                rotateRelative(TTAutoConstants.ROTATE_TO_SHOOT, TTAutoConstants.ROTATE_POWER);

                updateTelemetry("Cycle " + cycleCount, "Shooting balls");
                shootBalls();
            }

            telemetry.addData("Status", "COMPLETE!");
            telemetry.addData("Total Cycles", cycleCount);
            telemetry.addData("Final Heading", "%.1f degrees", odometry.getHeading());
            telemetry.addData("Final Drive Pos", "%.2f inches", odometry.driveDistance);
            telemetry.addData("Final Strafe Pos", "%.2f inches", odometry.strafeDistance);
            telemetry.update();

            sleep(15000);
            break;
        }
    }

    private void updateTelemetry(String phase, String action) {
        telemetry.clear();
        telemetry.addData("Phase", phase);
        telemetry.addData("Action", action);
        telemetry.addData("Heading", "%.1f°", odometry.getHeading());
        telemetry.addData("Drive Pos", "%.1f in", odometry.driveDistance);
        telemetry.addData("Strafe Pos", "%.1f in", odometry.strafeDistance);
        telemetry.update();
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
