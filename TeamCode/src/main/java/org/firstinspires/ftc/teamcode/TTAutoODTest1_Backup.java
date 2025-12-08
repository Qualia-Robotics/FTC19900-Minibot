package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Test 1: Backup to Shooting Position
 * Tests only the initial backup movement to verify odometry calibration
 */
@Autonomous(name = "Test1: Backup 68in", group = "Tests")
@Disabled
public class TTAutoODTest1_Backup extends LinearOpMode {
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

        telemetry.addData("Status", "Ready to backup " + TTAutoConstants.BACKUP_TO_SHOOT + " inches");
        telemetry.addData("Test", "Verify robot moves straight backward");
        telemetry.update();

        waitForStart();

        // Initialize odometry
        odometry.initialize(true);

        telemetry.addData("Status", "Starting backup...");
        telemetry.update();

        while (opModeIsActive()) {
            // Test: Back up to shooting position
            odometry.drive(-TTAutoConstants.BACKUP_TO_SHOOT, TTAutoConstants.DRIVE_POWER, 0);
            telemetry.addData("Status", "Backup complete!");
            telemetry.addData("Final Position", "%.2f inches", odometry.driveDistance);
            telemetry.addData("Final Heading", "%.1f degrees", odometry.getHeading());
            telemetry.update();

            // Wait so user can verify position
            sleep(10000);
            break;
        }
    }
}
