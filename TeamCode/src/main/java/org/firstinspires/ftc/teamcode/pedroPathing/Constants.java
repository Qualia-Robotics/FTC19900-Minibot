package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.DriveEncoderConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static double ROBOT_MAX_POWER = 0.7;
    public static double ROBOT_WEIGHT_KG = 10.5;
    public static double ROBOT_WIDTH_INCHES = 15.75;
    public static double ROBOT_LENGTH_INCHES = 17;
    public static double WHEEL_CIRCUMFERENCE_MM = 104;
    public static double WHEEL_CIRCUMFERENCE_INCHES = WHEEL_CIRCUMFERENCE_MM / 25.4;
    public static double FORWARD_TICKS_TO_INCHES = 0.010; // from tuning
    public static double STRAFE_TICKS_TO_INCHES = 0.020;  // from tuning
    public static double TURN_TICKS_TO_INCHES = 0.013;    // from tuning

    public static FollowerConstants followerConstants = new FollowerConstants()
            .automaticHoldEnd(true)
            .mass(ROBOT_WEIGHT_KG);

    public static MecanumConstants mecanumConstants = new MecanumConstants()
            .leftFrontMotorName("left_drive_front")
            .leftRearMotorName("left_drive_back")
            .rightFrontMotorName("right_drive_front")
            .rightRearMotorName("right_drive_back")
            .maxPower(ROBOT_MAX_POWER)
            .useBrakeModeInTeleOp(true);

    public static DriveEncoderConstants driveEncoderConstants = new DriveEncoderConstants()
            .leftFrontMotorName("left_drive_front")
            .leftRearMotorName("left_drive_back")
            .rightFrontMotorName("right_drive_front")
            .rightRearMotorName("right_drive_back")
            .robotWidth(ROBOT_WIDTH_INCHES)
            .robotLength(ROBOT_LENGTH_INCHES)
            .forwardTicksToInches(FORWARD_TICKS_TO_INCHES)
            .strafeTicksToInches(STRAFE_TICKS_TO_INCHES)
            .turnTicksToInches(TURN_TICKS_TO_INCHES);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(mecanumConstants)
                .driveEncoderLocalizer(driveEncoderConstants)
                .build();
    }
}
