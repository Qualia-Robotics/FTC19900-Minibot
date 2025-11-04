package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * This file contains a minimal example of a Linear "OpMode". An OpMode is a 'program' that runs
 * in either the autonomous or the TeleOp period of an FTC match. The names of OpModes appear on
 * the menu of the FTC Driver Station. When an selection is made from the menu, the corresponding
 * OpMode class is instantiated on the Robot Controller and executed.
 *
 * Remove the @Disabled annotation on the next line or two (if present) to add this OpMode to the
 * Driver Station OpMode list, or add a @Disabled annotation to prevent this OpMode from being
 * added to the Driver Station.
 */
@Autonomous(name = "Red Auto Android", group = "LionsSpark")
public class RedAutoAndroid extends LinearOpMode {
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;
    private DcMotorEx leftFlywheel;
    private DcMotorEx rightFlywheel;
    private DcMotor intake;
    private Limelight3A limelight;
    private Servo servo;

    // Setting our velocity targets. These values are in ticks per second!
    private static final int bankVelocity = 1200;
    private static final int farVelocity = 1500;
    private static final int maxVelocity = 2000;

    //Setting Toggle Booleans
    private static boolean ltPressedLast = false;
    private static boolean servoToggled = false;
    private static boolean rtPressedLast = false;
    private static boolean intakeToggled = false;
    private static boolean bPressedLast = false;

    //Assisting variables for distance calculations
    private static double targetOffsetAngle_Verticle;
    private static final double limelightMountAngleDegrees = 11;
    private static final double limelightLensHeightInches = 10.0;
    private static final double goalHeightInches = 29.5;
    private static double angleToGoalDegrees;
    private static double angleToGoalRadians;
    private static double distanceFromLimelightToGoalInches;

    //test
    private static int test = 0; //RPM

    //Assistance for auto alignment
    private static double Tx;
    private static boolean autoAim = false;

    //Boolean to check if limelight has vision
    private static boolean limelightIsValid = false;
    @Override
    public void runOpMode() {
        //Initializes
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");
        leftFlywheel = hardwareMap.get(DcMotorEx.class, "leftFlywheel");
        rightFlywheel = hardwareMap.get(DcMotorEx.class, "rightFlywheel");
        intake = hardwareMap.get(DcMotor.class, "intake");
        limelight = hardwareMap.get(Limelight3A.class, "LimeLight");
        servo = hardwareMap.get(Servo.class, "servo");

        // Establishing the direction and mode for the motors
        leftFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFlywheel.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        //Ensures the servo is active and ready
        // --- MOTOR BEHAVIOR --- //
        // Drivetrain and Climber set to BRAKE
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        servo.setPosition(0);
        limelight.pipelineSwitch(0);

        //defining auto aim switch


        float calculatedShootingVelocity;
        float areaForShooting;

        /*
         * Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.
         */
        limelight.start();
        waitForStart();
        if((opModeIsActive()))
        {
            frontLeft.setPower(.35);
            frontRight.setPower(-.35);
            backLeft.setPower(.35);
            backRight.setPower(-.35);
            leftFlywheel.setVelocity(1400.0);
            rightFlywheel.setVelocity(1400.0);
            sleep(2000);

            frontLeft.setPower(0.0);
            frontRight.setPower(0.0);
            backLeft.setPower(0.0);
            backRight.setPower(0.0);
            intake.setPower(-1);
            sleep(25);

            servo.setPosition(0);
            intake.setPower(1);
            sleep(5000);

            frontLeft.setPower(.35);
            frontRight.setPower(.35);
            backLeft.setPower(-.35);
            backRight.setPower(-.35);
            sleep(2000);

            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
            leftFlywheel.setVelocity(0.0);
            rightFlywheel.setVelocity(0.0);
        }

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.update();

        }
    }
}
