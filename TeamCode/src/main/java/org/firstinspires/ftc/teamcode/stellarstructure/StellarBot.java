package org.firstinspires.ftc.teamcode.stellarstructure;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class StellarBot {
	protected final Subsystem[] subsystems;

	public StellarBot(Subsystem... subsystems) {
		this.subsystems = subsystems;

		//add subsystems to scheduler
		for (Subsystem subsystem : subsystems) {
			Scheduler.getInstance().addSubsystem(subsystem);
		}
	}

	public final void init(HardwareMap hardwareMap) {
		//initialize all subsystems
		for (Subsystem subsystem : subsystems) {
			subsystem.init(hardwareMap);
		}
	}

	public final void setGamepads(Gamepad gamepad1, Gamepad gamepad2) {
		//set gamepads for all subsystems
		for (Subsystem subsystem : subsystems) {
			subsystem.setGamepads(gamepad1, gamepad2);
		}
	}

	public final void update() {
		//update triggers and directives
		Scheduler.getInstance().run();

		//update subsystems
		for (Subsystem subsystem : subsystems) {
			subsystem.update();
		}
	}

	@NonNull
	@Override
	public final String toString() {
		StringBuilder telemetry = new StringBuilder();

		for (Subsystem subsystem: subsystems) {
			telemetry.append(subsystem).append('\n');
		}

		telemetry.append(Scheduler.getInstance());

		return telemetry.toString();
	}

	public final void cancelAll() {
		Scheduler.getInstance().cancelAll();
	}
}