package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Tars Carousel Test", group = "Robot")
public class Paddles extends OpMode {
    public Servo paddles;
    public double paddlePosition = 0.0;

    @Override
    public void init() {
        paddles = hardwareMap.get(Servo.class, "carouselPaddles");
    }

    public void loop() {
        if (gamepad1.a) {
            paddlePosition = 0.0;
        } else if (gamepad1.b) {
            paddlePosition = 0.5;
        } else if (gamepad1.x) {
            paddlePosition = 1.0;
        }

        paddles.setPosition(paddlePosition);

        telemetry.addData("Paddle Position", paddlePosition);
        telemetry.update();
    }
}
