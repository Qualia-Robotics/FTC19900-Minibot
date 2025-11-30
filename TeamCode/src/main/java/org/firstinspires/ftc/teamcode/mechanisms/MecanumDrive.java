package org.firstinspires.ftc.teamcode.mechanisms;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class MecanumDrive {
    private DcMotor leftDriveFront;
    private DcMotor rightDriveFront;
    private DcMotor leftDriveBack;
    private DcMotor rightDriveBack;

    private double leftPowerFront;
    private double rightPowerFront;
    private double leftPowerBack;
    private double rightPowerBack;

    private double maxPower = 1.0;
    private double maxSpeed = 0.9;

    IMU imu;
    private double driveAngularOffset;

    public void initDriveMotor_leftFront(HardwareMap hwMap, String driveName) {
        leftDriveFront = hwMap.get(DcMotor.class, driveName);
        leftDriveFront.setDirection(DcMotor.Direction.REVERSE);
        leftDriveFront.setZeroPowerBehavior(BRAKE);
    }

    public void initDriveMotor_leftBack(HardwareMap hwMap, String driveName) {
        leftDriveBack = hwMap.get(DcMotor.class, driveName);
        leftDriveBack.setDirection(DcMotor.Direction.REVERSE);
        leftDriveBack.setZeroPowerBehavior(BRAKE);
    }

    public void initDriveMotor_RightFront(HardwareMap hwMap, String driveName) {
        rightDriveFront = hwMap.get(DcMotor.class, driveName);
        rightDriveFront.setDirection(DcMotor.Direction.FORWARD);
        rightDriveFront.setZeroPowerBehavior(BRAKE);
    }

    public void initDriveMotor_RightBack(HardwareMap hwMap, String driveName) {
        rightDriveBack = hwMap.get(DcMotor.class, driveName);
        rightDriveBack.setDirection(DcMotor.Direction.FORWARD);
        rightDriveBack.setZeroPowerBehavior(BRAKE);
    }

    public void initDriveMotor(HardwareMap hwMap, String LfFnt, String LfBk, String RtFnt, String RtBk) {
        initDriveMotor_leftFront(hwMap, LfFnt);
        initDriveMotor_leftBack(hwMap, LfBk);
        initDriveMotor_RightFront(hwMap, RtFnt);
        initDriveMotor_RightBack(hwMap, RtBk);
    }

    public void initRevIMU(HardwareMap hwMap, String imuName,
                           RevHubOrientationOnRobot.LogoFacingDirection logoDir,
                           RevHubOrientationOnRobot.UsbFacingDirection usbDir) {
        imu = hwMap.get(IMU.class, imuName);

        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(logoDir, usbDir);

        imu.initialize(new IMU.Parameters(RevOrientation));
    }

    public void drive(double forward, double strafe, double rotate) {
        leftPowerFront  = forward + strafe + rotate;
        rightPowerFront = forward - strafe - rotate;
        leftPowerBack   = forward - strafe + rotate;
        rightPowerBack  = forward + strafe - rotate;

        maxPower = Math.max(maxPower, Math.max(Math.abs(leftPowerFront), Math.max(Math.abs(rightPowerFront),
                Math.max(Math.abs(leftPowerBack), Math.abs(rightPowerBack)))));

        leftPowerFront  = maxSpeed * (leftPowerFront  / maxPower);
        rightPowerFront = maxSpeed * (rightPowerFront / maxPower);
        leftPowerBack   = maxSpeed * (leftPowerBack   / maxPower);
        rightPowerBack  = maxSpeed * (rightPowerBack  / maxPower);

        leftDriveFront.setPower(leftPowerFront);
        rightDriveFront.setPower(rightPowerFront);
        leftDriveBack.setPower(leftPowerBack);
        rightDriveBack.setPower(rightPowerBack);
    }

    public void driveFieldRelative(double forward, double strafe, double rotate) {
        double theta = Math.atan2(forward, strafe);
        double r = Math.hypot(strafe, forward);

        theta = AngleUnit.normalizeRadians(theta - this.getDriveHeading(AngleUnit.RADIANS));

        double newForward = r * Math.sin(theta);
        double newStrafe = r * Math.cos(theta);

        this.drive(newForward, newStrafe, rotate);
    }

    public void setDriveAngularOffset(double offset) {
        driveAngularOffset = offset;
    }

    public double getDriveAngularOffset(AngleUnit angleUnit) {
        if (angleUnit == AngleUnit.RADIANS)
            return driveAngularOffset * Math.PI / 180.0;
        else
            return driveAngularOffset;
    }

    public double getDriveHeading(AngleUnit angleUnit) {
        return imu.getRobotYawPitchRollAngles().getYaw(angleUnit) + this.getDriveAngularOffset(angleUnit);
    }

    public void resetDriveYaw() {
        imu.resetYaw();
    }

    public void setMaxPower(double maxPower) { this.maxPower = maxPower; }
    public void setMaxSpeed(double maxSpeed) { this.maxSpeed = maxSpeed; }
    public double getMaxPower() { return this.maxPower; }
    public double getMaxSpeed() { return this.maxSpeed; }

    public double getLeftPowerFront() { return leftPowerFront; }
    public double getLeftPowerBack() { return leftPowerBack; }
    public double getRightPowerFront() { return rightPowerFront; }
    public double getRightPowerBack() { return rightPowerFront; }
}