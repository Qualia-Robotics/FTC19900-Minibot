package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="AutoTT2B.java", group = "Fall2025")
//@Disabled
public class AutoTT2B extends LinearOpMode {

    private DcMotor frontleft = null;  //  Used to control the left front drive wheel
    private DcMotor frontright = null;  //  Used to control the right front drive wheel
    private DcMotor backleft = null;  //  Used to control the left back drive wheel
    private DcMotor backright = null;  //  Used to control the right back drive wheel
    private DcMotor leftlaunch = null;
    private DcMotor rightlaunch = null;
    private DcMotor index = null;
    private CRServo rotate = null;
    private CRServo rotate2 = null;

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

        backright.setDirection(DcMotorSimple.Direction.FORWARD);
        backleft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontleft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontright.setDirection(DcMotorSimple.Direction.FORWARD);
        waitForStart();
        while (opModeIsActive()) {
            backleft.setPower(0);
            backright.setPower(0);
            frontleft.setPower(0);
            frontright.setPower(0);
            sleep(2000);
            leftlaunch.setPower(0.89);
            rightlaunch.setPower(-0.89);
            sleep(500);
            backleft.setPower(-1);
            backright.setPower(-1);
            frontleft.setPower(-1);
            frontright.setPower(-1);
            sleep(500);
            backleft.setPower(0);
            backright.setPower(0);
            frontleft.setPower(0);
            frontright.setPower(0);
            sleep(200);
            index.setPower(-0.85);
            rotate.setPower(-1);
            rotate2.setPower(1);
            sleep(3700);
            index.setPower(0);
            rotate.setPower(0);
            rotate2.setPower(0);
            sleep(200);
            backleft.setPower(1);
            backright.setPower(-1);
            frontleft.setPower(1);
            frontright.setPower(-1);
            sleep(200);
            backleft.setPower(1);
            backright.setPower(-1);
            frontleft.setPower(-1);
            frontright.setPower(1);
            sleep(1000);
            backleft.setPower(0);
            backright.setPower(0);
            frontleft.setPower(0);
            frontright.setPower(0);
            sleep(1000000);
        }
    }
}