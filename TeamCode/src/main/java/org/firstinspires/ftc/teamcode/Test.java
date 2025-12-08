package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "Test", group = "Fall2025")
@Disabled
public class Test extends LinearOpMode {
    // private SimplifiedOdometryRobot robot = new SimplifiedOdometryRobot(this);
    private DcMotor frontleft; // Used to control the left front drive wheel
    private DcMotor frontright; // Used to control the right front drive wheel
    private DcMotor backleft; // Used to control the left back drive wheel
    private DcMotor backright; // Used to control the right back drive wheel
    private DcMotorEx leftlaunch;
    private DcMotorEx rightlaunch;

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
        leftlaunch = hardwareMap.get(DcMotorEx.class, "leftlaunch");
        rightlaunch = hardwareMap.get(DcMotorEx.class, "rightlaunch");
        index = hardwareMap.get(DcMotor.class, "index");
        rotate = hardwareMap.get(CRServo.class, "rotate");
        rotate2 = hardwareMap.get(CRServo.class, "rotate2");


        odometry = new SimplifiedOdometryRobotCustom(this, index, leftlaunch);


        backright.setDirection(DcMotorSimple.Direction.FORWARD);
        backleft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontleft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontright.setDirection(DcMotorSimple.Direction.FORWARD);
        waitForStart();

        // Initialize odometry system with starting heading of 45 degrees
        odometry.initialize(true);

        while (opModeIsActive()) {
odometry.strafe(5, 1, 0);
backleft.setPower(0);
backright.setPower(0);
frontleft.setPower(0);
frontright.setPower(0);
sleep(10000);
        }
    }
}