package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp

public class Tele_Op extends LinearOpMode {
    private DcMotor LeftDrive;
    private DcMotor RightDrive;

    private DcMotor Motor;


    @Override
    public void runOpMode() {
        LeftDrive = hardwareMap.get(DcMotor.class, "LeftDrive");
        RightDrive = hardwareMap.get(DcMotor.class, "RightDrive");
        Motor = hardwareMap.get(DcMotor.class, "Motor");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();


        while (opModeIsActive()) {
            //DRIVETRAIN CODE
            double leftAxis = -gamepad1.left_stick_y;
            double rightAxis = -gamepad1.right_stick_y;

            double leftPower = -leftAxis;
            double rightPower = rightAxis;
            double motpow = gamepad1.left_trigger;
            Motor.setPower(motpow);

            LeftDrive.setPower(leftPower);
            RightDrive.setPower(rightPower);
            if (gamepad1.triangle) {
                Motor.setPower(-0.8);
            }
            }
            telemetry.update();
        }
    }
}
