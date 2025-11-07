package org.firstinspires.ftc.team28420.util;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public final class Config {
    public final static class GamepadConf {
        public final static double LEFT_DEAD_ZONE = 0.2;
        public final static double RIGHT_DEAD_ZONE = 0.6;
        public final static double COEFFICIENT = 0.8;
    }

    public final static class WheelBaseConf {
        public final static int VELOCITY_COEFFICIENT = 4600;
        public final static double WHEEL_DIAMETER = 10.0;

        public final static String LEFT_TOP_MOTOR = "motorLT";
        public final static String RIGHT_TOP_MOTOR = "motorRT";
        public final static String LEFT_BOTTOM_MOTOR = "motorLB";
        public final static String RIGHT_BOTTOM_MOTOR = "motorRB";
    }

    public final static class GyroConf {
        public final static double[] CONST_ANGLES = {0, 45, 90, 135, 180, 225, 270, 315};

        public final static RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        public final static RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
    }

    public static final class Etc {
        public static Telemetry telemetry;
    }



}
