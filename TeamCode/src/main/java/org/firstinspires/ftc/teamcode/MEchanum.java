package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class OldRevTeleOp extends OpMode {
    private DcMotor rightFront;
    private DcMotor rightRear;
    private DcMotor leftFront;
    private DcMotor leftRear;

    @Override
    // runs before anything
    public void init() {
        // Initialize motors
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");

        // --- IMPORTANT MECANUM DIRECTION SETTINGS ---
        // For a standard Mecanum setup, the left side motors must be reversed
        // relative to the right side motors to maintain directional control.
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);

        // It's generally good practice to set motor modes for consistent power
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Initialized. Ready for Mecanum drive.");

    }


    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
    }

    // runs after hits start button
    @Override
    public void loop() {
        MecanumMovement();
        telemetry.update();
    }

    @Override
    public void stop() {
    }

    public void MecanumMovement() {
        boolean Status = true;

        // 1. Get the control inputs from the gamepad.
        // We invert the Y axis so that pushing UP is positive forward power.
        double drive = -gamepad1.left_stick_y;  // Forward/Backward (Axial)
        double strafe = gamepad1.left_stick_x; // Left/Right (Lateral)
        double turn = gamepad1.right_stick_x; // Turning/Rotation (Yaw)

        // 2. Apply the Mecanum math formula (vector sum).
        // LF = Drive + Strafe + Turn
        // RF = Drive - Strafe - Turn
        // LR = Drive - Strafe + Turn
        // RR = Drive + Strafe - Turn

        double leftFrontPower = drive + strafe + turn;
        double rightFrontPower = drive - strafe - turn;
        double leftRearPower = drive - strafe + turn;
        double rightRearPower = drive + strafe - turn;

        // 3. Normalize the power values.
        // Find the largest absolute value among the four powers.
        double maxPower = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        maxPower = Math.max(maxPower, Math.abs(leftRearPower));
        maxPower = Math.max(maxPower, Math.abs(rightRearPower));

        // If the max power is greater than 1.0, scale all powers down by that amount.
        if (maxPower > 1.0) {
            leftFrontPower /= maxPower;
            rightFrontPower /= maxPower;
            leftRearPower /= maxPower;
            rightRearPower /= maxPower;
        }

        // 4. Set the power to the motors.
        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftRear.setPower(leftRearPower);
        rightRear.setPower(rightRearPower);

        // Optional: Add telemetry to help with debugging
        telemetry.addData("Is on: " , Status);
        telemetry.addData("LF Power", "%.2f", leftFrontPower);
        telemetry.addData("RF Power", "%.2f", rightFrontPower);
        telemetry.addData("LR Power", "%.2f", leftRearPower);
        telemetry.addData("RR Power", "%.2f", rightRearPower);
    }
}