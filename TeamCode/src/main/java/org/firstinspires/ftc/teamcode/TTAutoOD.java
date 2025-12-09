package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.onbotjava.handlers.admin.ResetOnBotJava;

@Autonomous(name = "TTAutoOD.java", group = "Fall2025")
// @Disable
public class TTAutoOD extends LinearOpMode {
  // private SimplifiedOdometryRobot robot = new SimplifiedOdometryRobot(this);
  private DcMotor frontleft; // Used to control the left front drive wheel
  private DcMotor frontright; // Used to control the right front drive wheel
  private DcMotor backleft; // Used to control the left back drive wheel
  private DcMotor backright; // Used to control the right back drive wheel
  private DcMotor leftlaunch;
  private DcMotor rightlaunch;
  private DcMotor index;
  private CRServo rotate;
  private CRServo rotate2;

  private SimplifiedOdometryRobotCustom odometry;

  @Override
  public void runOpMode() {

    frontleft = hardwareMap.get(DcMotor.class, "frontleft");
    frontright = hardwareMap.get(DcMotor.class, "frontright");
    backleft = hardwareMap.get(DcMotor.class, "backleft");
    backright = hardwareMap.get(DcMotor.class, "backright");
    leftlaunch = hardwareMap.get(DcMotor.class, "leftlaunch");
    rightlaunch = hardwareMap.get(DcMotor.class, "rightlaunch");
    index = hardwareMap.get(DcMotor.class, "index");
    rotate = hardwareMap.get(CRServo.class, "rotate");
    rotate2 = hardwareMap.get(CRServo.class, "rotate2");

    odometry = new SimplifiedOdometryRobotCustom(this, index, rightlaunch); // leftlaunch was removed from parameters

    backright.setDirection(DcMotorSimple.Direction.FORWARD);
    backleft.setDirection(DcMotorSimple.Direction.REVERSE);
    frontleft.setDirection(DcMotorSimple.Direction.REVERSE);
    frontright.setDirection(DcMotorSimple.Direction.FORWARD);
    waitForStart();

    // Initialize odometry system with starting heading of 45 degrees
    odometry.initialize(true);

    while (opModeIsActive()) {
      // Step 1: Back up to shooting position
      odometry.drive(-TTAutoConstants.BACKUP_TO_SHOOT, TTAutoConstants.DRIVE_POWER, 0);

      // Step 2: Shoot pre-loaded balls
      shootBalls();

      // Loop through 3 ball groups with different strafe distances
      for (int i = 0; i < TTAutoConstants.STRAFE_DISTANCES.length; i++) {
        double strafeDistance = TTAutoConstants.STRAFE_DISTANCES[i];

        // Step 3: Rotate counterclockwise to collection angle
        rotateRelative(TTAutoConstants.ROTATE_TO_COLLECT, TTAutoConstants.ROTATE_POWER);

        // Step 4: Strafe left to ball group
        odometry.strafe(-strafeDistance, TTAutoConstants.DRIVE_POWER, 0);

        // Step 5: Drive forward while collecting balls
        driveAndCollect(TTAutoConstants.DRIVE_TO_COLLECT, TTAutoConstants.DRIVE_POWER);

        // Step 6: Back up to shooting lane
        odometry.drive(-TTAutoConstants.DRIVE_TO_COLLECT, TTAutoConstants.DRIVE_POWER, 0);

        // Step 7: Strafe right (return to shooting position)
        odometry.strafe(strafeDistance, TTAutoConstants.DRIVE_POWER, 0);

        // Step 8: Rotate clockwise to shooting angle
        rotateRelative(TTAutoConstants.ROTATE_TO_SHOOT, TTAutoConstants.ROTATE_POWER);

        // Step 9: Shoot collected balls
        shootBalls();
      }

      // Exit autonomous loop
      break;
    }
  }

  // Helper method to shoot balls
  private void shootBalls() {
    leftlaunch.setPower(TTAutoConstants.LEFT_LAUNCH_POWER);
    rightlaunch.setPower(TTAutoConstants.RIGHT_LAUNCH_POWER);
    sleep(TTAutoConstants.LAUNCHER_SPINUP_TIME);
    index.setPower(TTAutoConstants.INDEX_SHOOT_POWER);
    rotate.setPower(TTAutoConstants.ROTATE_SERVO_POWER);
    rotate2.setPower(TTAutoConstants.ROTATE2_SERVO_POWER);
    sleep(TTAutoConstants.SHOOTING_DURATION);

    // Stop all mechanisms
    leftlaunch.setPower(0);
    rightlaunch.setPower(0);
    index.setPower(0);
    rotate.setPower(0);
    rotate2.setPower(0);
  }

  // Helper method to drive forward and collect balls
  private void driveAndCollect(double inches, double power) {
    // Start intake mechanism
    index.setPower(TTAutoConstants.INDEX_COLLECT_POWER);

    // Drive forward while collecting
    odometry.drive(inches, power, 0);

    // Stop intake
    index.setPower(0);
  }

  // Helper method to rotate relative to current heading
  private void rotateRelative(double degrees, double power) {
    // Get current heading from odometry
    double currentHeading = odometry.getHeading();

    // Calculate target heading
    double targetHeading = currentHeading + degrees;

    // Normalize to 0-360 range
    while (targetHeading >= 360)
      targetHeading -= 360;
    while (targetHeading < 0)
      targetHeading += 360;

    // Turn to target heading
    odometry.turnTo(targetHeading, power, TTAutoConstants.ROTATION_TOLERANCE);
  }
}