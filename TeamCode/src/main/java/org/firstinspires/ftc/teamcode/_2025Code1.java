package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "_2025Code1 (Blocks to Java)")
public class _2025Code1 extends LinearOpMode {

  private DcMotor back_left;
  private DcMotor front_left;
  private DcMotor back_right;
  private DcMotor front_right;

  /**
   * This sample contains the bare minimum Blocks for any regular OpMode. The 3 blue Comment
   * Blocks show where to place Initialization code (runs once, after touching the DS INIT
   * button, and before touching the DS Start arrow), Run code (runs once, after touching
   * Start), and Loop code (runs repeatedly while the OpMode is active, namely not Stopped).
   */
  @Override
  public void runOpMode() {
    boolean speedtoggle;
    double WheelSpeedDivisor;
    int mode;
    int makeshiftpowervariable;
    float vertical;
    float horizontal;
    float pivot;

    back_left = hardwareMap.get(DcMotor.class, "back_left");
    front_left = hardwareMap.get(DcMotor.class, "front_left");
    back_right = hardwareMap.get(DcMotor.class, "back_right");
    front_right = hardwareMap.get(DcMotor.class, "front_right");

    // INIT
    speedtoggle = true;
    WheelSpeedDivisor = 1.15;
    mode = 0;
    makeshiftpowervariable = 0;
    back_left.setDirection(DcMotor.Direction.REVERSE);
    front_left.setDirection(DcMotor.Direction.REVERSE);
    waitForStart();
    if (opModeIsActive()) {
      // WHILE LOOP (Main)
      while (opModeIsActive()) {
        // Detect if the mode isn't 1
        if (mode == 0) {
          // Lift and more setup
          vertical = -gamepad1.right_stick_y;
          horizontal = -gamepad1.right_stick_x;
          pivot = -gamepad1.left_stick_x;
          // Movement Handler
          // Used to be -+,+-
          back_left.setPower((-pivot + vertical + horizontal) / WheelSpeedDivisor);
          back_right.setPower((pivot + (vertical - horizontal)) / WheelSpeedDivisor);
          front_left.setPower((-pivot + (vertical - horizontal)) / WheelSpeedDivisor);
          front_right.setPower((pivot + vertical + horizontal) / WheelSpeedDivisor);
          // Extra Features, note that these are still in testing and probably include a bunch of bugs.
          if (gamepad1.x && gamepad1.y) {
            // Killswitch
            terminateOpModeNow();
          }
          if (gamepad1.dpad_up) {
            gamepad1.setLedColor(0, 1, 0, 676);
            gamepad1.rumble(1, 0, 676);
            WheelSpeedDivisor = 1;
          }
          if (gamepad1.dpad_down) {
            gamepad1.setLedColor(1, 0, 0, 676);
            gamepad1.rumble(1, 0, 676);
            WheelSpeedDivisor = 2;
          }
          telemetry.update();
        }
      }
    }
  }
}
