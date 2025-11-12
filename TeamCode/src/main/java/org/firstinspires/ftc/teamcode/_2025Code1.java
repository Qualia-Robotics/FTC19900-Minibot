package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.CM;
import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.MM;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name = "_2025Code1")
public class _2025Code1 extends OpMode {
  GoBildaPinpointDriver odo;
  private DcMotor back_left;
  private DcMotor front_left;
  private DcMotor back_right;
  private DcMotor front_right;
  boolean speedtoggle;
  double WheelSpeedDivisor;
  int mode;
  float vertical;
  float horizontal;
  float pivot;

  @Override
  public void init() {
    odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
    speedtoggle = true;
    WheelSpeedDivisor = 1.15;
    mode = 0;
    back_left.setDirection(DcMotor.Direction.REVERSE);
    front_left.setDirection(DcMotor.Direction.REVERSE);
    odo.setOffsets(-84.0, -168.0, MM);//tuned for 3110-0002-0001, may be changed
    odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
    odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

    //Starting Position
    odo.resetPosAndIMU();
    Pose2D startingPosition= new Pose2D(DistanceUnit.MM, -923.925, 1601.47, AngleUnit.RADIANS, 0);
  }
  public void moveRobot() {
    double forward = -gamepad1.right_stick_y;
    double strafe = gamepad1.right_stick_x;
    double rotate = gamepad1.left_stick_x;

    Pose2D pos = odo.getPosition();
    double heading = pos.getHeading(AngleUnit.RADIANS);

    double sinAngle = Math.sin((Math.PI / 2) - heading);
    double cosAngle = Math.cos((Math.PI / 2) - heading);

    double globalStrafe = -forward * sinAngle + strafe * cosAngle;
    double globalForward = forward * cosAngle + strafe * sinAngle;

    double[] newWheelSpeeds = new double[4];

    newWheelSpeeds[0] = globalForward + globalStrafe + rotate;
    newWheelSpeeds[1] = globalForward - globalStrafe - rotate;
    newWheelSpeeds[2] = globalForward - globalStrafe + rotate;
    newWheelSpeeds[3] = globalForward + globalStrafe - rotate;

    //Set wheel power
    front_left.setPower(newWheelSpeeds[0] / WheelSpeedDivisor);
    front_right.setPower(newWheelSpeeds[1] / WheelSpeedDivisor);
    back_left.setPower(newWheelSpeeds[2] / WheelSpeedDivisor);
    back_right.setPower(newWheelSpeeds[3] / WheelSpeedDivisor);

    telemetry.addData("XPos: ", pos.getX(DistanceUnit.MM));
    telemetry.addData("YPos: ", pos.getY(DistanceUnit.MM));
  }
  public void loop() {
    moveRobot();

    Pose2D pos = odo.getPosition();
    //Extra Features, note that these are still in testing and probably include a bunch of bugs.
    if (gamepad1.x && gamepad1.y) {
      // Killswitch
      terminateOpModeNow();
    }
    if (gamepad1.dpad_up && WheelSpeedDivisor == 2) {
      gamepad1.setLedColor(0, 1, 0, 676);
      gamepad1.rumble(1, 0, 676);
      WheelSpeedDivisor = 1;
    }
    if (gamepad1.dpad_down && WheelSpeedDivisor == 1) {
      gamepad1.setLedColor(1, 0, 0, 676);
      gamepad1.rumble(1, 0, 676);
      WheelSpeedDivisor = 2;
    }
    telemetry.update();
  }
}