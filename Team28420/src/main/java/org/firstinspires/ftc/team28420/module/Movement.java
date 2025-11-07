package org.firstinspires.ftc.team28420.module;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.team28420.types.PolarVector;
import org.firstinspires.ftc.team28420.types.WheelsRatio;
import org.firstinspires.ftc.team28420.util.Config;

public class Movement {

    private final DcMotorEx leftFront, rightFront, leftBack, rightBack;

    public Movement(DcMotorEx leftFront, DcMotorEx rightFront, DcMotorEx leftBack, DcMotorEx rightBack) {
        this.leftFront = leftFront;
        this.rightFront = rightFront;
        this.leftBack = leftBack;
        this.rightBack = rightBack;
    }

    public void setMotorsTargetPosition(WheelsRatio<Double> wheelsRatio) {
        WheelsRatio<Integer> wheelsRatioInteger = wheelsRatio.toInt(1);
        leftFront.setTargetPosition(wheelsRatioInteger.getLeftTop());
        rightFront.setTargetPosition(wheelsRatioInteger.getRightTop());
        leftBack.setTargetPosition(wheelsRatioInteger.getLeftBottom());
        rightBack.setTargetPosition(wheelsRatioInteger.getRightBottom());
    }

    public void setMotorsPowerRatios(WheelsRatio<Double> wheelsRatio) {
        Config.Etc.telemetry.addData("left front", wheelsRatio.getLeftTop());
        Config.Etc.telemetry.addData("right front", wheelsRatio.getRightTop());
        Config.Etc.telemetry.addData("left bottom", wheelsRatio.getLeftBottom());
        Config.Etc.telemetry.addData("right bottom", wheelsRatio.getRightBottom());


        leftFront.setVelocity(wheelsRatio.getLeftTop());
        rightFront.setVelocity(wheelsRatio.getRightTop());
        leftBack.setVelocity(wheelsRatio.getLeftBottom());
        rightBack.setVelocity(wheelsRatio.getRightBottom());
    }

    public void setMotorsVelocityRatios(WheelsRatio<Double> wheelsRatio, int velocityMult) {
        leftFront.setVelocity(wheelsRatio.getLeftBottom() * velocityMult);
        rightFront.setVelocity(wheelsRatio.getRightTop() * velocityMult);
        leftBack.setVelocity(wheelsRatio.getLeftBottom() * velocityMult);
        rightBack.setVelocity(wheelsRatio.getRightBottom() * velocityMult);
    }


    public void setMotorsMode(DcMotor.RunMode mode) {
        leftFront.setMode(mode);
        rightFront.setMode(mode);
        leftBack.setMode(mode);
        rightBack.setMode(mode);
    }

    public void setMotorsZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        leftFront.setZeroPowerBehavior(behavior);
        rightFront.setZeroPowerBehavior(behavior);
        leftBack.setZeroPowerBehavior(behavior);
        rightBack.setZeroPowerBehavior(behavior);
    }

    public boolean isBusy() {
        return leftFront.isBusy() || rightFront.isBusy() || leftBack.isBusy() || rightBack.isBusy();
    }

    public void brake() {
        setMotorsPowerRatios(WheelsRatio.ZERO);
    }

    public void setup() {
        //rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        //rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        setMotorsMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setMotorsZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public WheelsRatio vectorToRatios(PolarVector vector, double turn) {
        double sin = Math.sin(vector.getTheta() - Math.PI / 4);
        double cos = Math.cos(vector.getTheta() - Math.PI / 4);
        double max = Math.max(Math.abs(sin), Math.abs(cos));

        double lf = vector.getAbs() * cos / max + turn;
        double rf = vector.getAbs() * sin / max - turn;
        double lb = vector.getAbs() * sin / max + turn;
        double rb = vector.getAbs() * cos / max - turn;

        if ((vector.getAbs() + Math.abs(turn)) > 1) {
            lf /= vector.getAbs() + Math.abs(turn);
            rf /= vector.getAbs() + Math.abs(turn);
            lb /= vector.getAbs() + Math.abs(turn);
            rb /= vector.getAbs() + Math.abs(turn);
        }

        return new WheelsRatio<Double>(lf, rf, lb, rb);
    }

}
