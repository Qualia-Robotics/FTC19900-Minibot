
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp (name ="MotorTester",group="1(Main OpModes")
public class MotorTester extends LinearOpMode {

    private DcMotor Motor;


    @Override
    public void runOpMode() {
        Motor = hardwareMap.get(DcMotor.class, "Motor");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();


        while (opModeIsActive()) {

            double motpow = gamepad1.left_trigger;
            Motor.setPower(motpow);
            /*if (gamepad1.triangle) {
                Motor.setPower(-0.8);
            }
            if (gamepad1.circle) {
                Motor.setPower(0.8);
            }*/
        }
        telemetry.update();
    }
}

