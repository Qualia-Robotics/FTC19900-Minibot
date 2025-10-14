package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * This OpMode is used for driving motors during prototyping.
 */
@TeleOp(name="Drive Flywheel")
public class FlyWheelOpMode extends OpMode {

    private DcMotor flywheel1;
    private DcMotor flywheel2;

    private DcMotor intake;

    private DcMotor motor;

    @Override
    public void init() {
        flywheel1 = hardwareMap.get(DcMotor.class, "flywheel1");
        flywheel2 = hardwareMap.get(DcMotor.class, "flywheel2");
        intake = hardwareMap.get(DcMotor.class, "intake");
        motor = hardwareMap.get(DcMotor.class, "motor");

        flywheel1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flywheel2.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        flywheel1.setPower(1.0);
        flywheel2.setPower(1.0);
        intake.setPower(1.0);
        motor.setPower(1.0);

    }
}
