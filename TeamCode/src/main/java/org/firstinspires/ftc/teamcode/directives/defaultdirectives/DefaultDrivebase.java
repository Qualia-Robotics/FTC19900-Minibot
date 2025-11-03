package org.firstinspires.ftc.teamcode.directives.defaultdirectives;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.Drivebase;
import org.firstinspires.ftc.teamcode.stellarstructure.runnables.DefaultDirective;

public class DefaultDrivebase extends DefaultDirective {
	private final double cardinalSpeed, turnSpeed;
	private final Gamepad gamepad;
	private final Drivebase drivebase;

	public DefaultDrivebase(Drivebase drivebase, Gamepad gamepad, double cardinalSpeed, double turnSpeed) {
		super(drivebase);

		this.cardinalSpeed = cardinalSpeed;
		this.turnSpeed = turnSpeed;
		this.gamepad = gamepad;
		this.drivebase = drivebase;
	}

	@Override
	public void update() {
		double max, axial, lateral, yaw;
		double leftFrontPower, rightFrontPower, leftBackPower, rightBackPower;
		axial = -gamepad.left_stick_y * cardinalSpeed;  // Note: pushing stick forward gives negative value
		lateral = gamepad.left_stick_x * cardinalSpeed;
		yaw = gamepad.right_stick_x * turnSpeed;

		// combine the joystick requests for each axis-motion to determine each wheel's power
		leftFrontPower = axial + lateral + yaw;
		rightFrontPower = axial - lateral - yaw;
		leftBackPower = axial - lateral + yaw;
		rightBackPower = axial + lateral - yaw;

		// normalize the values so no wheel power exceeds 100%
		// this ensures that the robot maintains the desired motion
		max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
		max = Math.max(max, Math.abs(leftBackPower));
		max = Math.max(max, Math.abs(rightBackPower));

		// maintain desired motion
		if (max > 1.0) {
			leftFrontPower /= max;
			rightFrontPower /= max;
			leftBackPower /= max;
			rightBackPower /= max;
		}

		// send calculated power to wheels
		drivebase.setDrivePower(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
	}
}