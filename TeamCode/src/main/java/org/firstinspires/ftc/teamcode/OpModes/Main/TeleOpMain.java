//NEED TO UPDATE FLYWHEEL SYNC AFTER KICKER MOVEMENT

package org.firstinspires.ftc.teamcode.OpModes.Main;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.OpModes.Main.Components.Robot;

@TeleOp(name = "TeleOpMain", group = "Linear OpMode")
public class TeleOpMain extends LinearOpMode {
    // Robot instance containing all components
    private Robot robot;

    @Override
    public void runOpMode() {
        // Initialize robot (all components initialized within)
        robot = new Robot();
        robot.initialize(hardwareMap, telemetry, this);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // Update launcher (flywheel)
            robot.updateLauncher();

            // Update turret alignment - exit OpMode if limelight not connected (matches original behavior)
            if (!robot.updateTurret()) {
                return;
            }
            telemetry.update();

            // Update drive train
            double forward = -gamepad1.left_stick_y;
            double right = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;
            robot.updateDriveTrain(forward, right, rotate, gamepad1.cross);
            telemetry.update();

            // Update spindexer
            boolean gamepadA = gamepad1.a;
            boolean gamepadB = gamepad1.b;
            boolean gamepadX = gamepad1.x;
            boolean gamepadY = gamepad1.y;
            boolean gamepadLeftBumper = gamepad1.left_bumper;

            // Handle shooting button - start flywheel when X is pressed (before shooting sequence)
            if (gamepadX && !robot.getSpindexer().isPrevX()) {
                robot.getLauncher().setSpinning(true);
                robot.updateLauncher(); // Update flywheel power immediately
            }

            robot.updateSpindexer(gamepadA, gamepadB, gamepadX, gamepadY, gamepadLeftBumper);

            // Stop flywheel after 3 shots
            if (robot.getSpindexer().shouldStopFlywheel()) {
                robot.getLauncher().setSpinning(false);
            }

            // Update parking (uses Lift component)
            robot.updateParking(gamepad1.dpad_up, gamepad1.dpad_down);

            telemetry.update();
            idle();
        }
    }
}
