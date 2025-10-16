package org.firstinspires.ftc.team28420.types;

public class PolarVector {
    public static PolarVector fromPos(Position pos) {
        return new PolarVector(Math.atan2(pos.y, pos.x), Math.hypot(pos.x, pos.y));
    }

    private double theta, abs;

    public double getTheta() {
        return theta;
    }

    public void setTheta(double theta) {
        this.theta = theta;
    }

    public double getAbs() {
        return abs;
    }

    public void setAbs(double abs) {
        this.abs = abs;
    }

    public PolarVector(double theta, double abs) {
        this.theta = theta;
        this.abs = abs;
    }

    public PolarVector rotate(double angle) {
        theta += angle;
        theta %= Math.PI * 2;
        return this;
    }

}
