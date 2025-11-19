package org.firstinspires.ftc.teamcode.PID;

public class FlywheelPID {

    private double kP, kI, kD;

    private double integral = 0;
    private double lastError = 0;
    private long lastTime = 0;

    public FlywheelPID(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.lastTime = System.nanoTime();
    }

    public double calculate(double target, double current) {

        double error = target - current;

        long now = System.nanoTime();
        double dt = (now - lastTime) / 1e9;
        lastTime = now;

        integral += error * dt;
        double derivative = (error - lastError) / dt;
        lastError = error;

        return (kP * error) + (kI * integral) + (kD * derivative);
    }
}
