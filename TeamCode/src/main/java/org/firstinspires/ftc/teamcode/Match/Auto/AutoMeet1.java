/*
 * MIT License
 *
 * Copyright (c) 2024 ParkCircus Productions; All Rights Reserved
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.Match.Auto;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


@Autonomous(name="Meet 1 Auto", group="Match", preselectTeleOp="TeleOpPreviewEvent")

public class AutoMeet1 extends LinearOpMode {
    private static final String TAG = AutoMeet1.class.getSimpleName();

    // Enumerations for Alliance and Position
    enum Alliance {
        RED,
        BLUE
    }

    enum Position {
        POS1,
        POS2,
        POS3
    }

    // Enumeration for the Finite State Machine (FSM)
    enum RobotState {
        SCAN_OBELISK,
        SCAN_GOAL,
        LAUNCH,
        TRAVEL,
        LEAVE,
        COMPLETE
    }

    // Hardware for Mecanum Drivetrain
    // --- Hardware Declarations (Using components expected from the original code) ---
    private DcMotorEx motorLeftFront = null;
    private DcMotorEx motorLeftBack = null;
    private DcMotorEx motorRightFront = null;
    private DcMotorEx motorRightBack = null;
    private IMU imu = null; // Assuming IMU is used for heading
    private TouchSensor touchSensor = null;
    private VoltageSensor batterySensor;
    static final double TICKS_PER_MOTOR_REV = 28.0; // SWYFT v2 drive
    static final double WHEEL_DIAMETER_INCHES = 3.3856; // 86 mm
    static final double DRIVE_GEAR_REDUCTION = 12.7;
    static final double TICKS_PER_INCH =
            (TICKS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);
    // private static final double TICKS_PER_INCH = 33.4308289114498; // SWYFT Drive v2; goBILDA 5203 series, 12.7:1, 86 mm

    static final double DRIVE_SPEED = 0.5;
    static final double HEADING_GAIN = 0.03; // P-gain for straightness
    // Variables for Alliance and Position
    private Alliance alliance = Alliance.RED; // Default to Red Alliance
    private Position position = Position.POS1; // Default to Position 1

    // Initializing the FSM state
    private RobotState currentState = RobotState.LEAVE;

    // --- VISION COMPONENTS ---
    private static final int DESIRED_TAG_ID = -1;     // Set to -1 for ANY tag.
    private VisionPortal visionPortal = null; // Initialize to null for safety
    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag = null;

    // A simple class to represent a Waypoint in the autonomous path
    private class Waypoint {
        public double x;
        public double y;
        public double heading;

        public Waypoint(double x, double y, double heading) {
            this.x = x;
            this.y = y;
            this.heading = heading;
        }
    }

    // Waypoint entities for each starting position.
    // Replace these placeholder values with your actual coordinates.
    private final Waypoint redPos1Waypoint = new Waypoint(-36.0, 60.0, 90.0);
    private final Waypoint redPos2Waypoint = new Waypoint(-12.0, 60.0, 90.0);
    private final Waypoint redPos3Waypoint = new Waypoint(12.0, 60.0, 90.0);

    private final Waypoint bluePos1Waypoint = new Waypoint(-36.0, -60.0, -90.0);
    private final Waypoint bluePos2Waypoint = new Waypoint(-12.0, -60.0, -90.0);
    private final Waypoint bluePos3Waypoint = new Waypoint(12.0, -60.0, -90.0);

    @Override
    public void runOpMode() {
        // --- Hardware Initialization ---
        // Assuming your IMU is named "imu" in the configuration
        imu = hardwareMap.get(IMU.class, "imu");

        // --- B. Define Hub Orientation on Robot ---
        // 1. Create the orientation object using the specified directions
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                // Logo Direction: The side of the hub with the REV logo
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                // USB Direction: The side of the hub with the USB-C port
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        );
        // --- C. Create and Apply IMU Parameters ---
        // 2. Create the IMU Parameters object and pass the orientation
        IMU.Parameters parameters = new IMU.Parameters(orientationOnRobot);
        // 3. Initialize the IMU with the parameters
        // This process automatically calibrates and configures the sensor.
        imu.initialize(parameters);
        // --- D. Mecanum motor initialization
        motorLeftFront = hardwareMap.get(DcMotorEx.class, "motorLeftFront");
        motorLeftBack = hardwareMap.get(DcMotorEx.class, "motorLeftBack");
        motorRightFront = hardwareMap.get(DcMotorEx.class, "motorRightFront");
        motorRightBack = hardwareMap.get(DcMotorEx.class, "motorRightBack");

        motorLeftFront.setDirection(DcMotorEx.Direction.REVERSE);
        motorLeftBack.setDirection(DcMotorEx.Direction.REVERSE);
        motorRightFront.setDirection(DcMotorEx.Direction.FORWARD);
        motorRightBack.setDirection(DcMotorEx.Direction.FORWARD);

        motorLeftFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorLeftBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorRightFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorRightBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // --- Driver Hub Pre-Match Selection ---

        telemetry.addData("Status", "Ready for Selection");
        telemetry.update();
        sleep(3000);

        while (!isStarted() && !isStopRequested()) {
            // Alliance Selection
            if (gamepad1.b) {
                alliance = Alliance.BLUE;
            } else if (gamepad1.x) {
                alliance = Alliance.RED;
            }

            // Position Selection
            if (gamepad1.dpad_up) {
                position = Position.POS1;
            } else if (gamepad1.dpad_right) {
                position = Position.POS2;
            } else if (gamepad1.dpad_down) {
                position = Position.POS3;
            }

            telemetry.addData("Alliance", "Press B|O for Blue, X|□ for Red");
            telemetry.addData("Position", "Press D-pad Up/Right/Down for POS1/POS2/POS3");
            telemetry.addData("Current Selection", "Alliance: %s, Position: %s", alliance.toString(), position.toString());
            telemetry.update();
        }

        waitForStart();
        imu.resetYaw(); // safe practice if robot orientation is orthogonal to field, i.e. heading = 0

        // --- Autonomous Execution ---

        if (opModeIsActive()) {

            telemetry.addData("Status", "Executing Autonomous Routine");
            telemetry.addData("Routine", "Alliance: %s, Position: %s", alliance.toString(), position.toString());
            telemetry.update();

            // Select the correct waypoint based on user input
            Waypoint selectedWaypoint;
            switch (alliance) {
                case RED:
                    switch (position) {
                        case POS1:
                            selectedWaypoint = redPos1Waypoint;
                            break;
                        case POS2:
                            selectedWaypoint = redPos2Waypoint;
                            break;
                        case POS3:
                            selectedWaypoint = redPos3Waypoint;
                            break;
                        default:
                            selectedWaypoint = redPos1Waypoint; // Fallback
                            break;
                    }
                    break;
                case BLUE:
                    switch (position) {
                        case POS1:
                            selectedWaypoint = bluePos1Waypoint;
                            break;
                        case POS2:
                            selectedWaypoint = bluePos2Waypoint;
                            break;
                        case POS3:
                            selectedWaypoint = bluePos3Waypoint;
                            break;
                        default:
                            selectedWaypoint = bluePos1Waypoint; // Fallback
                            break;
                    }
                    break;
                default:
                    selectedWaypoint = redPos1Waypoint; // Fallback
                    break;
            }

            // Call the main autonomous routine with the selected waypoint
            runAutonomousRoutine(selectedWaypoint);
            sleep(1000);
        }
    }

    // --- Main Autonomous Routine with FSM ---
    private void runAutonomousRoutine(Waypoint startPoint) {

        telemetry.addData("Executing Path", "Starting from (%.1f, %.1f) with heading %.1f", startPoint.x, startPoint.y, startPoint.heading);
        telemetry.update();

        // Main FSM loop. The opModeIsActive() check allows the driver to stop the OpMode at any time.
        while (opModeIsActive() && currentState != RobotState.COMPLETE) {

            telemetry.addData("Current State", currentState.toString());
            telemetry.update();

            switch (currentState) {
                case SCAN_OBELISK:
                    // Placeholder: Code to move to a position to scan the obelisk
                    // and use computer vision to determine its location.
                    sleep(2000); // Wait for vision processing

                    // After scanning, transition to the next state
                    currentState = RobotState.SCAN_GOAL;
                    break;

                case SCAN_GOAL:
                    // Placeholder: Code to move to the determined goal position
                    // and verify its location before launch.
                    //sleep(1500); // Wait for position verification

                    // After scanning the goal, transition to launch
                    currentState = RobotState.LAUNCH;
                    break;

                case LAUNCH:
                    //launch(1.0); // launch power = 1.0

                    // After the launch action is initiated, wait for it to finish
                    currentState = RobotState.TRAVEL;
                    break;

                case TRAVEL:
                    /*
                    if(drive(DRIVE_SPEED, -4, DistanceUnit.INCH, 1)){
                        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        currentState = RobotState.ROTATING;
                    }
                     */
                    currentState = RobotState.COMPLETE;
                    break;

                case LEAVE:
                    // speed = 0.5, distance = 12.0, heading = 0.0 -- for test purposes only
                    driveMecanum(0.5, 12.0, 0.0);
                    currentState = RobotState.COMPLETE;
                    break;

                case COMPLETE:
                    // This state has no action. The `while` loop condition will handle termination.
                    break;
                default:
                    throw new IllegalStateException("FSM: unexpected value: " + currentState);
            }
        }
    }

    /**
     * Drives the robot straight forward or backward using encoders for distance
     * and IMU for heading correction.
     * @param speed Base power (0.0 to 1.0, direction determined by distance)
     * @param distanceInches Target distance (+ for forward, - for backward)
     * @param desiredHeading The constant heading (Yaw) to maintain (e.g., 0.0 degrees)
     */
    public void driveMecanum(double speed, double distanceInches, double desiredHeading) {
        if (!opModeIsActive()) return;

        // 1. Calculate Target Ticks
        int moveCounts = (int) (distanceInches * TICKS_PER_INCH);

        int frontLeftTarget = motorLeftFront.getCurrentPosition() + moveCounts;
        int frontRightTarget = motorRightFront.getCurrentPosition() + moveCounts;
        int backLeftTarget = motorLeftBack.getCurrentPosition() + moveCounts;
        int backRightTarget = motorRightBack.getCurrentPosition() + moveCounts;

        // 2. Set Targets and Run Mode
        motorLeftFront.setTargetPosition(frontLeftTarget);
        motorRightFront.setTargetPosition(frontRightTarget);
        motorLeftBack.setTargetPosition(backLeftTarget);
        motorRightBack.setTargetPosition(backRightTarget);

        // Use RUN_TO_POSITION
        motorLeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Start the movement
        double absSpeed = Math.abs(speed);

        // 3. Loop until all motors reach position (Encoder Logic)
        while (opModeIsActive() &&
                (motorLeftFront.isBusy() || motorRightFront.isBusy() ||
                        motorLeftBack.isBusy() || motorRightBack.isBusy())) {

            // 4. IMU Correction Logic
            // Get current heading (Yaw)
            double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            // Calculate error
            double error = desiredHeading - currentHeading;
            while (error > 180)  error -= 360; // Normalize error to +/- 180
            while (error <= -180) error += 360;

            // Calculate turn correction
            double turnCorrection = error * HEADING_GAIN;

            // Determine if moving forward or backward to apply correction correctly
            double direction = Math.signum(distanceInches);

            // Apply correction
            // When moving forward (direction = +1): TurnCorrection is subtracted from Left and added to Right to correct yaw
            double leftPower = absSpeed * direction - turnCorrection * direction;
            double rightPower = absSpeed * direction + turnCorrection * direction;

            // Set Power (using RUN_TO_POSITION ignores power setting except to determine velocity/torque)
            // Note: For simple movements like this, simply setting the power based on the correction often works fine.
            motorLeftFront.setPower(leftPower);
            motorLeftBack.setPower(leftPower);
            motorRightFront.setPower(rightPower);
            motorRightBack.setPower(rightPower);
        }

        // 5. Stop and Reset
        motorLeftFront.setPower(0);
        motorRightFront.setPower(0);
        motorLeftBack.setPower(0);
        motorRightBack.setPower(0);

        // Switch back to RUN_USING_ENCODER for TeleOp or next movement
        motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Initializes the AprilTag processor and Vision Portal.
     */
    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(2);

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }


    /**
     * Helper method to start the launching mechanism.
     * Replace the placeholder code with your actual launcher control logic.
     */
    private void launch(double power) {
        // Example: Set the launcher motor power
        // launcherMotor.setPower(1.0);
    }

    /**
     * Helper method to check if the launch is complete.
     * Replace the placeholder code with your sensor or timer-based logic.
     * @return true if the launch action is finished, false otherwise.
     */
    private boolean isLaunchComplete() {
        // Example: Check if a sensor detects the game element has been launched
        // or if a timer has elapsed.
        // return (launcherMotor.getPower() == 0);
        return true; // Placeholder for now
    }

}

