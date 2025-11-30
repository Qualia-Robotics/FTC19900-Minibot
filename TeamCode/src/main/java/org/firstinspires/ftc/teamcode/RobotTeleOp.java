package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.mechanisms.IntakeMotor;
import org.firstinspires.ftc.teamcode.mechanisms.Launcher;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

/**
 * TeleOp program for an FTC robot with:
 *  - Four-motor Mecanum wheel drivetrain
 *  - One launcher motor using velocity control
 *  - Two feeder CRServos
 *  - Intake motor
 *
 * CONTROLS:
 *  Left stick Y = forward/backward (up = forward, down = backward)
 *  Left stick X = strafe (left = left, right = right)
 *  Right stick X = rotate (left = CCW, right = CW)
 *
 *  Right bumper = launcher to fire one close shot
 *  Right trigger = launcher to fire one far shot
 *
 *  Left bumper  = intake forward
 *  Left trigger = intake stop
 *  D-pad Left   = intake reverse
 */

@TeleOp
public class RobotTeleOp extends OpMode {
    private final double DRIVE_MAX_SPEED = 0.9;
    private final double INAKE_POWER = 0.75;
    private final double INTAKE_PANIC_TIME = 0.1;

    private final double ClOSE_LAUNCH_TARGET_VELOCITY = 1300;
    private final double CLOSE_LAUNCH_MIN_VELOCITY    = 1275;
    private final double FAR_LAUNCH_TARGET_VELOCITY   = 1600;
    private final double FAR_LAUNCH_MIN_VELOCITY      = 1575;

    private final double FEEDER_RUN_SECONDS = 0.10;
    private final double LAUNCH_COOLOFF_SECONDS = 0.20;

    private final double GAMEPAD_TRIGGER_PRESS_THRESHOLD = 0.5;

    // === Drivetrain motors ===
    MecanumDrive mecanumDrive = new MecanumDrive();

    // === Intake ===
    IntakeMotor intakeMotor = new IntakeMotor();

    // === Launcher and feeders ===
    Launcher launcher = new Launcher();

    // === Run timer & Misc. ===
    private ElapsedTime runtime= new ElapsedTime();
    private enum Alliance { BLUE, RED, NONE }
    private Alliance alliance = Alliance.NONE;
    private boolean isIMURequested = false;

    @Override
    public void init() {
        /* === Drivetrain setup === */
        mecanumDrive.initDriveMotor(hardwareMap, "left_drive_front","left_drive_back",
                "right_drive_front", "right_drive_back");
        mecanumDrive.setMaxSpeed(DRIVE_MAX_SPEED);

        /* === IMU setup === */
        mecanumDrive.initRevIMU(hardwareMap, "imu",
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);
        mecanumDrive.resetDriveYaw();

        /* === Intake setup === */
        intakeMotor.init(hardwareMap, "intake");
        intakeMotor.setPower(INAKE_POWER);
        intakeMotor.setPanicTime(INTAKE_PANIC_TIME);

        /* === Launcher setup === */
        launcher.init(hardwareMap, "launcher", "left_feeder", "right_feeder");
        launcher.setLaucherCloseVelocity(ClOSE_LAUNCH_TARGET_VELOCITY, CLOSE_LAUNCH_MIN_VELOCITY);
        launcher.setLaucherFarVelocity(FAR_LAUNCH_TARGET_VELOCITY, FAR_LAUNCH_MIN_VELOCITY);
        launcher.setLauncherCoolOffSec(LAUNCH_COOLOFF_SECONDS);
        launcher.setFeederRunSec(FEEDER_RUN_SECONDS);
    }

    @Override
    public void init_loop() {
        if (gamepad1.xWasPressed())
            alliance = Alliance.BLUE;
        else if (gamepad1.circleWasPressed())
            alliance = Alliance.RED;

        if (gamepad1.triangleWasPressed())
            isIMURequested = true;
        else if (gamepad1.crossWasPressed())
            isIMURequested = false;

        if (!isIMURequested)
            mecanumDrive.setDriveAngularOffset(0.0);
        else {
            if (alliance == Alliance.BLUE)
                mecanumDrive.setDriveAngularOffset(-90.0);
            else if (alliance == Alliance.RED)
                mecanumDrive.setDriveAngularOffset(90.0);
            else
                mecanumDrive.setDriveAngularOffset(0.0);
        }

        telemetry.addData("Status", "Initialized\n");
        telemetry.addData("Alliance", "Press Square/Circle button to select Blue/Red team\n");
        telemetry.addData("Use RevIMU", "Press Triangle/Cross to choose Field/Robot Orientation\n");
        telemetry.addData("Alliance", alliance == Alliance.BLUE ? "BLUE" : alliance == Alliance.RED ? "RED" : "NONE");
        telemetry.addData("IMURequested", isIMURequested ? "YES" : "NO");
        telemetry.addData("Robot Heading", mecanumDrive.getDriveHeading(AngleUnit.DEGREES));
        telemetry.update();
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        // === Drive Control ===
        double forward = -gamepad1.left_stick_y;
        double strafe  = gamepad1.left_stick_x;
        double rotate  = gamepad1.right_stick_x;

        if (!isIMURequested)
            mecanumDrive.drive(forward, strafe, rotate);
        else
            mecanumDrive.driveFieldRelative(forward, strafe, rotate);

        // === Intake Motor ===
        boolean intakeOn  = gamepad1.left_bumper;
        boolean intakeOff = (gamepad1.left_trigger > GAMEPAD_TRIGGER_PRESS_THRESHOLD);
        boolean panic     = gamepad1.dpad_left;

        intakeMotor.run(intakeOn, intakeOff, panic);

        // === Launcher ===
        boolean closeShotRequested = gamepad1.right_bumper;
        boolean farShotRequested   = (gamepad1.right_trigger > GAMEPAD_TRIGGER_PRESS_THRESHOLD);

        launcher.launch(closeShotRequested, farShotRequested);

        // === Status Output ===
        telemetry.addData("Alliance", alliance == Alliance.BLUE ? "BLUE" : alliance == Alliance.RED ? "RED" : "NONE");
        telemetry.addData("IMURequested", isIMURequested ? "YES\n" : "NO\n");
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Front Motor Power", "left (%.2f), right (%.2f)", mecanumDrive.getLeftPowerFront(), mecanumDrive.getRightPowerFront());
        telemetry.addData("Back Motor Power", "left (%.2f), right (%.2f)", mecanumDrive.getLeftPowerBack(), mecanumDrive.getRightPowerBack());
        telemetry.addData("Launcher State", launcher.getState());
        telemetry.addData("Launcher Speed", launcher.getLaucherVelocity());
        telemetry.addData("Intake State", intakeMotor.getState());
        telemetry.addData("Intake Direction", intakeMotor.getDirection());
        telemetry.addData("Intake Power", intakeMotor.getPower());
        telemetry.addData("Robot Heading", mecanumDrive.getDriveHeading(AngleUnit.DEGREES));
        telemetry.update();
    }
}
