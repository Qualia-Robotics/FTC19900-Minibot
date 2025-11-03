package org.firstinspires.ftc.teamcode.directives.defaultdirectives;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.directives.PulseTransferLever;
import org.firstinspires.ftc.teamcode.stellarstructure.Trigger;
import org.firstinspires.ftc.teamcode.stellarstructure.conditions.GamepadButtonMap;
import org.firstinspires.ftc.teamcode.stellarstructure.conditions.StatefulCondition;
import org.firstinspires.ftc.teamcode.stellarstructure.hardwaremapwrappers.StellarServo;
import org.firstinspires.ftc.teamcode.stellarstructure.runnables.DefaultDirective;
import org.firstinspires.ftc.teamcode.subsystems.LeverTransfer;

public class DefaultLeverTransfer extends DefaultDirective {
	//todo: implement starting conditions and directives and procedures
	public DefaultLeverTransfer(LeverTransfer leverTransfer, Gamepad gamepad, StellarServo leverTransferServo) {
		super(leverTransfer);

		addTrigger(new Trigger(
				new StatefulCondition(
						new GamepadButtonMap(gamepad, GamepadButtonMap.Button.DPAD_UP),
						StatefulCondition.Edge.RISING //On initial press
				),
				() -> {
					leverTransfer.setLeverPositionIsUp(true);
					leverTransfer.updateServoPosition();
				}
		));

		addTrigger(new Trigger(
				new StatefulCondition(
						new GamepadButtonMap(gamepad, GamepadButtonMap.Button.DPAD_DOWN),
						StatefulCondition.Edge.RISING //On initial press
				),
				() -> {
					leverTransfer.setLeverPositionIsUp(false);
					leverTransfer.updateServoPosition();
				}
		));

		addTrigger(new Trigger(
				new StatefulCondition(
						new GamepadButtonMap(gamepad, GamepadButtonMap.Button.DPAD_LEFT),
						StatefulCondition.Edge.RISING //On initial press
				),
				() -> {
					// up down up
					new PulseTransferLever().schedule();
				}
		));
	}
}