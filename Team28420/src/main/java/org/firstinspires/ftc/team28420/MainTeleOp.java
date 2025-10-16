package org.firstinspires.ftc.team28420;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.team28420.module.Movement;
import org.firstinspires.ftc.team28420.types.PolarVector;
import org.firstinspires.ftc.team28420.types.Position;
import org.firstinspires.ftc.team28420.util.Config;

@TeleOp(name = "Main", group = "prod")
public class MainTeleOp extends LinearOpMode {

    public Movement mov = null;

    @Override
    public void runOpMode() {
        initialize();
        setup();

        waitForStart();
        while (opModeIsActive()) {
            mov.setMotorsVelocities(mov.vectorToRatios(PolarVector.fromPos(
                new Position(
                    Math.abs(gamepad1.left_stick_x) > Config.GAMEPAD_LEFT_DEAD_ZONE ? gamepad1.left_stick_x * Config.GAMEPAD_COEFFICIENT: 0,
                    Math.abs(gamepad1.left_stick_y) > Config.GAMEPAD_LEFT_DEAD_ZONE ? -gamepad1.left_stick_y * Config.GAMEPAD_COEFFICIENT : 0
                )),  Math.abs(gamepad1.right_stick_x) > Config.GAMEPAD_RIGHT_DEAD_ZONE ? gamepad1.right_stick_x * Config.GAMEPAD_COEFFICIENT : 0)
            );
        }
    }

    private void initialize() {
        Config.Etc.telemetry = telemetry;

        mov = new Movement(hardwareMap.get(DcMotorEx.class, Config.LEFT_TOP_MOTOR),
                hardwareMap.get(DcMotorEx.class, Config.RIGHT_TOP_MOTOR),
                hardwareMap.get(DcMotorEx.class, Config.LEFT_BOTTOM_MOTOR),
                hardwareMap.get(DcMotorEx.class, Config.RIGHT_BOTTOM_MOTOR));
    }

    private void setup() {
        mov.setMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
