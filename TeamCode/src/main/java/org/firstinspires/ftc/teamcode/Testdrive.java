package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import android.os.Debug;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TestdriveOpMode Class", group = "rev0/tests")
//@Disabled
public class Testdrive extends OpMode {
    // THIS IS where we put our public variables that are stored in the oppmodes class
    public int timer = 0;
    public double LauncherMaxSpeed = 1075; // The speed that the launcher motor wll spin at when ready, and the speed that we want when it puts the ball into the chanmber
    public double LauncherSpeedLaunch = 1050; // the speed at witch the launcher will deplay the ball

    final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    final double FULL_SPEED = 1.0;
    final double FLIPPER_UP = 0.0;
    final double FLIPPER_DOWN = 0.55;
    //THIS IS where we declare our OpMode Mappings
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotorEx launcher = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;
    private Servo flipper = null;

    double leftPower;
    double rightPower;

    @Override
    public void init() {
        // this code runs when the user preses start !ONCE!

        // Initialize Hardware Mappings
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");
        flipper = hardwareMap.get(Servo.class,"flipper");
        // then set directions and stuff
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        launcher.setDirection(DcMotor.Direction.FORWARD);
        leftFeeder.setDirection(CRServo.Direction.REVERSE);
        rightFeeder.setDirection(CRServo.Direction.FORWARD);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftDrive.setZeroPowerBehavior(BRAKE);
        rightDrive.setZeroPowerBehavior(BRAKE);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        //increse timer
        timer++;

        // this is where you putt all the code for the robots op mode when it is initialized
        arcadeDrive(-gamepad1.left_stick_y, gamepad1.right_stick_x);


        if (gamepad1.a) {
            launcher.setVelocity(LauncherMaxSpeed);
            if ((Math.abs(launcher.getVelocity()) >= LauncherSpeedLaunch) && timer >= 500) {
                leftFeeder.setPower(FULL_SPEED);
                rightFeeder.setPower(FULL_SPEED);
                timer = 0;
            }
        } else {
            launcher.setVelocity(0);
            leftFeeder.setPower(0);
            rightFeeder.setPower(0);
        }


        telemetry.addData("motorSpeed", Math.abs(launcher.getVelocity()));

        if (gamepad1.left_bumper) {
         flipper.setPosition(FLIPPER_UP);
            telemetry.addLine("test 1");

        }
        if (gamepad1.right_bumper) {
            flipper.setPosition(FLIPPER_DOWN);
            telemetry.addLine("test 2");
        }
    }

    void arcadeDrive(double forward, double rotate) {
        leftPower = forward + rotate;
        rightPower = forward - rotate;

        /*
         * Send calculated power to wheels
         */
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
    }



    //CUSTOM METHODS
}