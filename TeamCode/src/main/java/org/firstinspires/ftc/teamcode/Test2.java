package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "Test2", group = "Fall2025")
@Disabled
public class Test2 extends LinearOpMode {

    private DcMotor frontleft, frontright, backleft, backright;
    private DcMotor driveEncoder, strafeEncoder;

    private SimplifiedOdometryRobotCustom odometry;

    @Override
    public void runOpMode() {
        // Hardware Map
        frontleft = hardwareMap.get(DcMotor.class, "frontleft");
        frontright = hardwareMap.get(DcMotor.class, "frontright");
        backleft = hardwareMap.get(DcMotor.class, "backleft");
        backright = hardwareMap.get(DcMotor.class, "backright");

        driveEncoder = frontleft;   // test using frontleft motor as encoder
        strafeEncoder = frontright; // test using frontright motor as strafe encoder

        // Motor directions
        frontleft.setDirection(DcMotorSimple.Direction.REVERSE);
        backleft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontright.setDirection(DcMotorSimple.Direction.FORWARD);
        backright.setDirection(DcMotorSimple.Direction.FORWARD);

        odometry = new SimplifiedOdometryRobotCustom(this, driveEncoder, strafeEncoder);

        telemetry.addLine("Initializing odometry...");
        telemetry.update();
        odometry.initialize(true);

        telemetry.addLine("Waiting for start...");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            telemetry.addLine("Starting diagnostic movements");
            telemetry.update();

            // Step 1: Drive forward 10 inches
            telemetry.addLine("Driving forward 10 inches");
            telemetry.update();
            odometry.drive(10, 0.5, 0);

            telemetry.addLine("Drive complete. Heading: " + odometry.getHeading());
            telemetry.update();
            sleep(1000);

            // Step 2: Strafe left 5 inches
            telemetry.addLine("Strafing left 5 inches");
            telemetry.update();
            odometry.strafe(5, 0.5, 0);

            telemetry.addLine("Strafe complete. Heading: " + odometry.getHeading());
            telemetry.update();
            sleep(1000);

            // Step 3: Turn 45 degrees CCW
            telemetry.addLine("Turning 45 degrees CCW");
            telemetry.update();
            odometry.turnTo(45, 0.5, 0);

            telemetry.addLine("Turn complete. Heading: " + odometry.getHeading());
            telemetry.update();
            sleep(1000);

            telemetry.addLine("Diagnostic finished");
            telemetry.update();
        }
    }
}
