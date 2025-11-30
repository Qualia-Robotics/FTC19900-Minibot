package org.firstinspires.ftc.teamcode.mechanisms;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Launcher {
    private DcMotorEx launcher;
    private CRServo leftFeeder;
    private CRServo rightFeeder;

    private final double STOP_SPEED = 0.0;
    private final double FULL_SPEED = 1.0;

    private double closeTargetVelocity = 1300;
    private double closeMinVelocity    = 1275;
    private double farTargetVelocity   = 1650;
    private double farMinVelocity      = 1625;

    private double feederRunSec = 0.10;
    private double launcherCoolOffSec = 0.20;

    // === Launcher state machine ===
    private enum LaunchState { IDLE, SPIN_UP_F, SPIN_UP_C, LAUNCH, LAUNCHING, COOL_OFF }
    private LaunchState launchState;

    private ElapsedTime feederTimer = new ElapsedTime();
    private ElapsedTime launchTimer = new ElapsedTime();

    public void init(HardwareMap hwMap, String flyWheel, String leftFeed, String rightFeed) {
        launcher = hwMap.get(DcMotorEx.class, flyWheel);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setVelocity(STOP_SPEED);
        launcher.setDirection(DcMotorSimple.Direction.REVERSE);
        launcher.setZeroPowerBehavior(BRAKE);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(300, 0, 0, 10));

        leftFeeder = hwMap.get(CRServo.class, "left_feeder");
        rightFeeder = hwMap.get(CRServo.class, "right_feeder");
        leftFeeder.setPower(STOP_SPEED);
        rightFeeder.setPower(STOP_SPEED);
        rightFeeder.setDirection(DcMotorSimple.Direction.REVERSE);

        launchState = Launcher.LaunchState.IDLE;
    }

    public void launch(boolean closeShotRequested, boolean farShotRequested) {
        switch (launchState) {
            case IDLE:
                if (closeShotRequested)
                    launchState = LaunchState.SPIN_UP_C;
                else if (farShotRequested)
                    launchState = LaunchState.SPIN_UP_F;
                else
                    launcher.setVelocity(STOP_SPEED);
                break;
            case SPIN_UP_C:
                launcher.setVelocity(closeTargetVelocity);
                if (launcher.getVelocity() > closeMinVelocity)
                    launchState = LaunchState.LAUNCH;
                break;
            case SPIN_UP_F:
                launcher.setVelocity(farTargetVelocity);
                if (launcher.getVelocity() > farMinVelocity)
                    launchState = LaunchState.LAUNCH;
                break;
            case LAUNCH:
                leftFeeder.setPower(FULL_SPEED);
                rightFeeder.setPower(FULL_SPEED);
                feederTimer.reset();
                launchState = LaunchState.LAUNCHING;
                break;
            case LAUNCHING:
                if (feederTimer.seconds() > feederRunSec) {
                    leftFeeder.setPower(STOP_SPEED);
                    rightFeeder.setPower(STOP_SPEED);
                    launchTimer.reset();
                    launchState = LaunchState.COOL_OFF;
                }
                break;
            case COOL_OFF:
                if (launchTimer.seconds() > launcherCoolOffSec)
                    launchState = LaunchState.IDLE;
                break;
        }
    }

    public String getState() {
        if (launchState == LaunchState.SPIN_UP_C)
            return "SPIN_UP_C";
        else if (launchState == LaunchState.SPIN_UP_F)
            return "SPIN_UP_F";
        else if (launchState == LaunchState.LAUNCH)
            return "LAUNCH";
        else if (launchState == LaunchState.LAUNCHING)
            return "LAUNCHING";
        else if (launchState == LaunchState.COOL_OFF)
            return "COOL_OFF";
        else
            return "IDLE";
    }

    public double getLaucherVelocity() {
        return launcher.getVelocity();
    }

    public void setLaucherCloseVelocity(double target, double min) {
        closeTargetVelocity = target;
        closeMinVelocity = min;
    }

    public void setLaucherFarVelocity(double target, double min) {
        farTargetVelocity = target;
        farMinVelocity = min;
    }

    public void setFeederRunSec (double second) {
        feederRunSec = second;
    }

    public void setLauncherCoolOffSec (double second) {
        launcherCoolOffSec = second;
    }
}
