package org.firstinspires.ftc.teamcode.stellarstructure;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.stellarstructure.runnables.DefaultDirective;

public abstract class Subsystem {
	private DefaultDirective defaultDirective;
	public abstract void init(HardwareMap hardwareMap);
	public abstract void setGamepads(Gamepad gamepad1, Gamepad gamepad2);
	public abstract void update();

	public final void setDefaultDirective(DefaultDirective defaultDirective) {
		this.defaultDirective = defaultDirective;
	}

	public final DefaultDirective getDefaultDirective() {
		return defaultDirective;
	}
}