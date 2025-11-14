package org.firstinspires.ftc.teamcode.Impulse;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp
public class IntakeTest extends OpMode {
    public DcMotor Intake2;
    boolean motorRunning = false;
    boolean lastButtonState = false;

    public void init(){
        Intake2 = hardwareMap.get(DcMotor.class, "intake");
    }
    public void loop(){
        if (gamepad1.a && !lastButtonState) {
            motorRunning = !motorRunning;
        }
        Intake2.setPower(motorRunning ? 1 : 0.0);
        lastButtonState = gamepad1.a;
    }
}
