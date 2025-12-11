package org.firstinspires.ftc.teamcode.OpModes.Main.Components;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ProgrammingBoard.ProgrammingBoardOTHER;

public class Flywheel {
    private ProgrammingBoardOTHER board;
    private Telemetry telemetry;
    
    private double flywheelPower = 0.8; // starting power
    private boolean spinning = false;   // flywheel state

    public void initialize(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        board = new ProgrammingBoardOTHER();
        board.initializeComponents(hardwareMap);
        telemetry.addData("Status", "Initialized. Flywheel power: " + flywheelPower);
        telemetry.update();
    }

    public void initialize(HardwareMap hardwareMap, Telemetry telemetry, double initialPower) {
        this.flywheelPower = Math.max(0.0, Math.min(1.0, initialPower));
        initialize(hardwareMap, telemetry);
    }

    public void update() {
        board.flyWheelMotor.setPower(spinning ? flywheelPower : 0);
        board.flyWheelMotor2.setPower(spinning ? flywheelPower : 0);
    }

    public void setSpinning(boolean spinning) {
        this.spinning = spinning;
    }

    public boolean isSpinning() {
        return spinning;
    }

    public void setPower(double power) {
        this.flywheelPower = Math.max(0.0, Math.min(1.0, power));
    }

    public double getPower() {
        return flywheelPower;
    }

    public void adjustPower(double increment) {
        flywheelPower = Math.max(0.0, Math.min(1.0, flywheelPower + increment));
    }
}

