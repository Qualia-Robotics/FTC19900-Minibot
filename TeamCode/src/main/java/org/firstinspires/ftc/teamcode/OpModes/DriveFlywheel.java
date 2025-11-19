package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake;
import org.firstinspires.ftc.teamcode.Mechanisms.Flywheel;
import org.firstinspires.ftc.teamcode.PID.FlywheelPID;

@TeleOp
public class DriveFlywheel extends OpMode {
    Drivetrain drive = new Drivetrain();
    Intake intake = new Intake();
    Flywheel flywheel = new Flywheel();
    private DcMotorEx FlywheelLMotor,FlywheelRMotor;
    FlywheelPID flyPID = new FlywheelPID(0.0008, 0.0, 0.0001);

    double targetRPM = 3000;



    @Override
    public void init() {
        drive.init(hardwareMap);
        intake.init(hardwareMap);
        flywheel.init(hardwareMap);
        FlywheelLMotor = hardwareMap.get(DcMotorEx.class, "flywheelL");
        FlywheelRMotor = hardwareMap.get(DcMotorEx.class,"flywheelR");
        FlywheelRMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }


    @Override
    public void loop() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;
        boolean input1 = gamepad1.a;
        boolean input2 = gamepad1.x;
        boolean input3 = gamepad1.b;
        double currentTicksPerSec = (FlywheelLMotor.getVelocity() + FlywheelRMotor.getVelocity())/2; // ticks/sec
        double currentRPM = (currentTicksPerSec * 60.0) / 28.0; // GoBILDA has 28 ticks/rev
        double pidOut = flyPID.calculate(targetRPM, currentRPM);

        // Clamp to motor power range
        pidOut = Math.max(0, Math.min(pidOut, 1));

        double flywheelPower = pidOut;

        // --- Telemetry ---
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Current RPM", currentRPM);
        telemetry.addData("Power", pidOut);
        telemetry.update();
        telemetry.addData("Flywheel", flywheelPower);


        drive.driveRobotRelative(y,x,turn);
        intake.NonStationary(input1);
        intake.stationary(input3);
        flywheel.doubleA(flywheelPower, input2);
    }
}