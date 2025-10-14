package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

@TeleOp(name="Gamepad Driving")
public class GamepadDriveTeleOp extends OpMode {
    MecanumDrive drive = new MecanumDrive(telemetry);

    @Override
    public void init() {
        drive.init(hardwareMap);
    }

    @Override
    public void loop() {

        double forward = -gamepad1.left_stick_y;
        double right = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        drive.drive(forward, right, rotate);
    }
}
