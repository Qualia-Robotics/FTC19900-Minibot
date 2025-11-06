package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
        (name = "Auto_Amulya", group = "Autonomous")
public class Auto_Amulya extends LinearOpMode {

    private DcMotor frontleft, frontright, backleft, backright;
    private DcMotor intake, outtakeleft, outtakeright;
    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 383.6;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;

    @Override
    public void runOpMode() {

        // ---- Drive Motors ----
        frontleft = hardwareMap.get(DcMotor.class, "frontleft");
        frontright = hardwareMap.get(DcMotor.class, "frontright");
        backleft = hardwareMap.get(DcMotor.class, "backleft");
        backright = hardwareMap.get(DcMotor.class, "backright");

        frontleft.setDirection(DcMotor.Direction.REVERSE);
        backleft.setDirection(DcMotor.Direction.REVERSE);
        frontright.setDirection(DcMotor.Direction.FORWARD);
        backright.setDirection(DcMotor.Direction.FORWARD);

        frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // ---- Intake / Outtake Motors ----
        intake = hardwareMap.get(DcMotor.class, "intake");
        outtakeleft = hardwareMap.get(DcMotor.class, "outtakeleft");
        outtakeright = hardwareMap.get(DcMotor.class, "outtakeright");

        outtakeleft.setDirection(DcMotor.Direction.FORWARD);
        outtakeright.setDirection(DcMotor.Direction.REVERSE);

        intake.setPower(0);
        outtakeleft.setPower(0);
        outtakeright.setPower(0);

        telemetry.addLine("Initialized — waiting for start");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {


            moveForward(12);


            moveBackward(12);


            strafeRight(12);

            strafeLeft(12);



            outtakeleft.setPower(0.25);
            outtakeright.setPower(0.25);
            telemetry.addLine("Running outtake...");
            telemetry.update();
            sleep(2000); // outtake runs for 2 seconds
            outtakeleft.setPower(0);
            outtakeright.setPower(0);
            sleep(500);


            intake.setPower(1.0);
            telemetry.addLine("Running intake...");
            telemetry.update();
            sleep(2000); // intake runs for 2 seconds
            intake.setPower(0);

            telemetry.addLine("Auto Complete!");
            telemetry.update();
        }
    }

    private void moveForward(double inches) {
        int move = (int)(inches * COUNTS_PER_INCH);
        frontleft.setTargetPosition(frontleft.getCurrentPosition() + move);
        frontright.setTargetPosition(frontright.getCurrentPosition() + move);
        backleft.setTargetPosition(backleft.getCurrentPosition() + move);
        backright.setTargetPosition(backright.getCurrentPosition() + move);

        runToPosition();
    }

    private void moveBackward(double inches) {
        int move = (int)(inches * COUNTS_PER_INCH);
        frontleft.setTargetPosition(frontleft.getCurrentPosition() - move);
        frontright.setTargetPosition(frontright.getCurrentPosition() - move);
        backleft.setTargetPosition(backleft.getCurrentPosition() - move);
        backright.setTargetPosition(backright.getCurrentPosition() - move);

        runToPosition();
    }

    private void strafeRight(double inches) {
        int move = (int)(inches * COUNTS_PER_INCH);
        frontleft.setTargetPosition(frontleft.getCurrentPosition() + move);
        frontright.setTargetPosition(frontright.getCurrentPosition() - move);
        backleft.setTargetPosition(backleft.getCurrentPosition() - move);
        backright.setTargetPosition(backright.getCurrentPosition() + move);

        runToPosition();
    }

    private void strafeLeft(double inches) {
        int move = (int)(inches * COUNTS_PER_INCH);
        frontleft.setTargetPosition(frontleft.getCurrentPosition() - move);
        frontright.setTargetPosition(frontright.getCurrentPosition() + move);
        backleft.setTargetPosition(backleft.getCurrentPosition() - move);
        backright.setTargetPosition(backright.getCurrentPosition() + move);

        runToPosition();
    }

    private void runToPosition() {
        frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontleft.setPower(DRIVE_SPEED);
        frontright.setPower(DRIVE_SPEED);
        backleft.setPower(DRIVE_SPEED);
        backright.setPower(DRIVE_SPEED);

        while (opModeIsActive() &&
                (frontleft.isBusy() && frontright.isBusy() && backleft.isBusy() && backright.isBusy())) {
            telemetry.addData("Moving...", "FL: %d FR: %d", frontleft.getCurrentPosition(), frontright.getCurrentPosition());
            telemetry.update();
        }

        frontleft.setPower(0);
        frontright.setPower(0);
        backleft.setPower(0);
        backright.setPower(0);

        frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sleep(250);
    }
}




