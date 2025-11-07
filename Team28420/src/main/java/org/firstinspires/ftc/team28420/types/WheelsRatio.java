package org.firstinspires.ftc.team28420.types;

import java.lang.Number;

public class WheelsRatio<T extends Number> {

    private T leftTop, rightTop, leftBottom, rightBottom;

    public void setLeftTop(T leftTop) {
        this.leftTop = leftTop;
    }

    public void setRightTop(T rightTop) {
        this.rightTop = rightTop;
    }

    public void setLeftBottom(T leftBottom) {
        this.leftBottom = leftBottom;
    }

    public void setRightBottom(T rightBottom) {
        this.rightBottom = rightBottom;
    }

    public T getLeftTop() {
        return leftTop;
    }

    public T getRightTop() {
        return rightTop;
    }

    public T getLeftBottom() {
        return leftBottom;
    }

    public T getRightBottom() {
        return rightBottom;
    }

    public WheelsRatio(T leftTop, T rightTop, T leftBottom, T rightBack) {
        this.leftTop = leftTop;
        this.rightTop = rightTop;
        this.leftBottom = leftBottom;
        this.rightBottom = rightBack;
    }

    public static final WheelsRatio<Double> ZERO = new WheelsRatio<Double>(0.0, 0.0, 0.0, 0.0);

    public WheelsRatio<Integer> toInt(double k) {
        return new WheelsRatio<>(
                (int) ((double) leftTop * k),
                (int) ((double) rightTop * k),
                (int) ((double) leftBottom * k),
                (int) ((double) rightBottom * k)
        );
    }

    @Override
    public String toString() {
        return "WheelsRatio{" +
                "leftFront=" + leftTop +
                ", rightFront=" + rightTop +
                ", leftBack=" + leftBottom +
                ", rightBack=" + rightBottom +
                '}';
    }
}