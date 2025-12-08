package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Test 3: Strafe Test
 * Tests strafing left and right with the three different distances
 */
@Autonomous(name = "Test3: Strafe Left/Right", group = "Tests")
@Disabled
public class TTAutoODTest3_Strafe extends LinearOpMode {
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

        telemetry.addData("Status", "Ready to test strafing");
        telemetry.addData("Test", "Testing all strafe distances");
        telemetry.update();

        waitForStart();

        // Initialize odometry
        odometry.initialize(true);

        while (opModeIsActive()) {


                telemetry.addData("Test", "Strafing left %.0f inches");
                telemetry.update();

                // Strafe left
                odometry.strafe(-32, TTAutoConstants.DRIVE_POWER, 0);
                sleep(2000);

                telemetry.addData("Test", "Strafing right %.0f inches");
                telemetry.update();

                // Strafe right (return)
                odometry.strafe(32, TTAutoConstants.DRIVE_POWER, 0);
                sleep(2000);

                telemetry.addData("Complete", "Distance %.0f test done");
                telemetry.addData("Strafe Position", "%.2f inches", odometry.strafeDistance);
                telemetry.update();
                sleep(2000);
            }

            telemetry.addData("Status", "All strafe tests complete!");
            telemetry.update();
            sleep(10000);
        }
    }

