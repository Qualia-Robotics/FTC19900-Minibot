package org.firstinspires.ftc.team28420;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.team28420.module.Gyro;
import org.firstinspires.ftc.team28420.module.Movement;
import org.firstinspires.ftc.team28420.types.PolarVector;
import org.firstinspires.ftc.team28420.types.Position;
import org.firstinspires.ftc.team28420.util.Config;

@TeleOp(name = "Main", group = "prod")
public class MainTeleOp extends LinearOpMode {

    public Movement mov = null;
    public Gyro gyro = null;

    @Override
    public void runOpMode() {
        initialize();
        setup();

        waitForStart();
        while (opModeIsActive()) {
            Config.Etc.telemetry.addData("left stick", gamepad1.left_stick_x);

            updateMotors();

            Config.Etc.telemetry.update();
        }
    }
    private void updateMotors() {
        mov.setMotorsVelocityRatios(mov.vectorToRatios(PolarVector.fromPos(
            new Position(
                    Math.abs(gamepad1.left_stick_x) > Config.GamepadConf.LEFT_DEAD_ZONE ? gamepad1.left_stick_x * Config.GamepadConf.COEFFICIENT : 0,
                    Math.abs(gamepad1.left_stick_y) > Config.GamepadConf.LEFT_DEAD_ZONE ? -gamepad1.left_stick_y * Config.GamepadConf.COEFFICIENT : 0
            )).rotate(Math.PI / 2),
            Math.abs(gamepad1.right_stick_x) > Config.GamepadConf.RIGHT_DEAD_ZONE ? gamepad1.right_stick_x * Config.GamepadConf.COEFFICIENT : 0),
            Config.WheelBaseConf.VELOCITY_COEFFICIENT
        );
    }

    private void initialize() {
        Config.Etc.telemetry = telemetry;

        gyro = new Gyro(hardwareMap.get(BHI260IMU.class,"imu"));

        mov = new Movement(hardwareMap.get(DcMotorEx.class, Config.WheelBaseConf.LEFT_TOP_MOTOR),
                hardwareMap.get(DcMotorEx.class, Config.WheelBaseConf.RIGHT_TOP_MOTOR),
                hardwareMap.get(DcMotorEx.class, Config.WheelBaseConf.LEFT_BOTTOM_MOTOR),
                hardwareMap.get(DcMotorEx.class, Config.WheelBaseConf.RIGHT_BOTTOM_MOTOR));
    }

    private void setup() {
        mov.setup();
        gyro.setup();
    }
}
