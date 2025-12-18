package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp

public class Tele_Op extends LinearOpMode {
    private DcMotor LeftDrive;
    private DcMotor RightDrive;


    @Override
    public void runOpMode() {
        LeftDrive = hardwareMap.get(DcMotor.class, "LeftDrive");
        RightDrive = hardwareMap.get(DcMotor.class, "RightDrive");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();


        while (opModeIsActive()) {
            //DRIVETRAIN CODE
            double leftAxis = -gamepad1.left_stick_y;
            double rightAxis = -gamepad1.right_stick_y;

            double leftPower = -leftAxis;
            double rightPower = rightAxis;

            LeftDrive.setPower(leftPower);
            RightDrive.setPower(rightPower);

            telemetry.update();
        }
    }
}
