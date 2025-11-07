package org.firstinspires.ftc.team28420;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team28420.module.Movement;
import org.firstinspires.ftc.team28420.types.WheelsRatio;
import org.firstinspires.ftc.team28420.util.Config;

@Autonomous(name = "auto", group = "prod")
public class MainAuto extends LinearOpMode {

    private Movement mov;

    @Override
    public void runOpMode() {
        initialize();
        setup();
        ElapsedTime ep = new ElapsedTime();

        waitForStart();
        mov.setMotorsPowerRatios(new WheelsRatio<>(
                100.0, 100.0, 100.0, 100.0
        ));

        while(ep.seconds() <= 20);

        stop();
    }

    private void initialize() {
        Config.Etc.telemetry = telemetry;

        mov = new Movement(hardwareMap.get(DcMotorEx.class, Config.WheelBaseConf.LEFT_TOP_MOTOR),
                hardwareMap.get(DcMotorEx.class, Config.WheelBaseConf.RIGHT_TOP_MOTOR),
                hardwareMap.get(DcMotorEx.class, Config.WheelBaseConf.LEFT_BOTTOM_MOTOR),
                hardwareMap.get(DcMotorEx.class, Config.WheelBaseConf.RIGHT_BOTTOM_MOTOR));
    }

    private void setup() {
        mov.setMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
