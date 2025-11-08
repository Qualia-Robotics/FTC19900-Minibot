package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class MotorTest extends LinearOpMode {
    private DcMotor rightDrive;

    @Override
    public void runOpMode() {
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                rightDrive.setPower(0.2);
            }
        }
    }
}
