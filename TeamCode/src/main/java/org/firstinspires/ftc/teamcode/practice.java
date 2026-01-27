package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class practice extends OpMode {
    @Override
    public void init() {
        int teamNumber = 19900;
        double motorSpeed = 1150;
        boolean reachedHeight = true;

        telemetry .addData("Team Number", teamNumber);
        telemetry .addData("Motor Speed", motorSpeed);
        telemetry .addData("Claw State", reachedHeight);
    }

    @Override
    public void loop() {

    }
}


