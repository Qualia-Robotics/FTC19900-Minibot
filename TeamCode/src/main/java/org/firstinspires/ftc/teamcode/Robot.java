package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import java.util.List;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot
{
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotorEx leftFlywheel;
    private DcMotorEx rightFlywheel;
    private DcMotor intake;
    private Servo servo;
    private Vision vision;
    private enum ShootState {
        IDLE,
        PRELOAD,
        SHOOT
    }
    ShootState shootState = ShootState.IDLE;
    public Robot(HardwareMap hardwareMap) {
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");
        leftFlywheel = hardwareMap.get(DcMotorEx.class, "leftFlywheel");
        rightFlywheel = hardwareMap.get(DcMotorEx.class, "rightFlywheel");
        intake = hardwareMap.get(DcMotor.class, "intake");
        servo = hardwareMap.get(Servo.class, "servo");
        vision = new Vision(hardwareMap);

    }
    public void init()
    {
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
        leftFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        servo.setPosition(.4);

        //set encoders
        leftFlywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFlywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFlywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFlywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public Servo getServo()
    {
        return this.servo;
    }
    public DcMotor getFrontLeft()
    {
        return this.frontLeft;
    }
    public DcMotor getFrontRight()
    {
        return this.frontRight;
    }
    public DcMotor getBackLeft()
    {
        return this.backLeft;
    }
    public DcMotor getBackRight()
    {
        return this.backRight;
    }
    public DcMotor getIntake()
    {
        return this.intake;
    }
    public DcMotorEx getLeftFlywheel()
    {
        return this.leftFlywheel;
    }
    public DcMotorEx getRightFlywheel()
    {
        return this.rightFlywheel;
    }
    public Limelight3A getLimelight()
    {
        return vision.getLimelight();
    }
    public Vision getVision()
    {
        return vision;
    }
    public void splitStickArcadeDrive() {
        //----------------//
        // MECANUM DRIVE  //
        //----------------//
        double rx = gamepad1.left_stick_y + gamepad1.right_stick_y;
        double x =  -gamepad1.left_stick_x;
        double y = gamepad1.right_stick_x;
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y + x - rx) / denominator;
        double backRightPower = (y - x - rx) / denominator;

        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);

    }

    public void autoAimArcadeDrive(double Tx) {
        double y;
        //----------------//
        // AUTOAIM DRIVE  //
        //----------------//
        if(!(vision.detecting())){
            y = gamepad1.right_stick_x;
        }else{
            if(Math.abs(Tx) < 3){
                y = 0;
            }else{
                y = Tx/40;
            }
        }




        double rx = gamepad1.left_stick_y + gamepad1.right_stick_y;
        double x =  -gamepad1.left_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y + x - rx) / denominator;
        double backRightPower = (y - x - rx) / denominator;

        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);

    }

    /**
     * Manual control for the Core Hex powered feeder and the agitator servo in the hopper
     */

    //Toggle for servo
    public void servoAndIntakeToggle(ElapsedTime shootTimer) {

        // Intake control (manual overrides)
        if (gamepad1.left_trigger > 0.1) {
            intake.setPower(1);
            return; // Skip shooting logic while intaking
        }
        if (gamepad1.left_bumper) {
            intake.setPower(-1);
            return;
        }

        // Shooting state machine
        switch (shootState) {

            case IDLE:
                if (gamepad1.right_trigger > 0.1) {
                    // Trigger pressed → start preload phase
                    shootState = ShootState.PRELOAD;
                    shootTimer.reset();
                }
                else {
                    // Default idle state
                    intake.setPower(0);
                    servo.setPosition(0.3);
                }
                break;

            case PRELOAD:
                // Run intake backwards briefly to "pull back" the ring
                intake.setPower(-1);

                // After 0.25 seconds, go to shooting phase
                if (shootTimer.seconds() > 0.15) {
                    shootState = ShootState.SHOOT;
                    servo.setPosition(0); // Fire
                }
                break;

            case SHOOT:
                if (gamepad1.right_trigger > 0.1) {
                    // Keep shooting while the trigger is held
                    intake.setPower(1);
                }
                else
                {
                    // Trigger released → stop everything
                    intake.setPower(0);
                    servo.setPosition(0.3);
                    shootState = ShootState.IDLE;
                }
                break;
        }
    }



    //Toggle for the shooter



    //Flywheel Code
    /**
     * This if/else statement contains the controls for the flywheel, both manual and auto.
     * Circle and Square will spin up ONLY the flywheel to the target velocity set.
     * The bumpers will activate the flywheel, Core Hex feeder, and servo to cycle a series of balls.
     */


    /**
     * The bank shot or near velocity is intended for launching balls touching or a few inches from the goal.
     * When running this function, the flywheel will spin up and the Core Hex will wait before balls can be fed.
     * The servo will spin until the bumper is released.
     */
    public void shoot(int targetVelocity)
    {
        double currentVelocity = leftFlywheel.getVelocity();
        if(currentVelocity < targetVelocity)
        {
            leftFlywheel.setPower(1);
            rightFlywheel.setPower(1);


        }
        else if((currentVelocity >= targetVelocity - 100) || (currentVelocity <= targetVelocity + 100))
        {
            leftFlywheel.setVelocity(targetVelocity);
            rightFlywheel.setVelocity(targetVelocity);

        }
        else
        {
            stopFlywheels();
        }
        //servo.setPower(-1);

    }

    /**
     * The far power velocity is intended for launching balls a few feet from the goal. It may require adjusting the deflector.
     * When running this function, the flywheel will spin up and the Core Hex will wait before balls can be fed.
     * The servo will spin until the bumper is released.
     */

    public void stopFlywheels() {
        ((DcMotorEx) leftFlywheel).setVelocity(0);
        ((DcMotorEx) rightFlywheel).setVelocity(0);

    }
    public void recordRobotTelemetry(Telemetry telemetry)
    {
        this.vision.recordVisionTelemetry(telemetry);
    }

}


