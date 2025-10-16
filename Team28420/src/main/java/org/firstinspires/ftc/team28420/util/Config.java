package org.firstinspires.ftc.team28420.util;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public final class Config {
    public static double GAMEPAD_LEFT_DEAD_ZONE = 0.2;
    public static double GAMEPAD_RIGHT_DEAD_ZONE = 0.6;
    public static double GAMEPAD_COEFFICIENT = 0.8;

    public static String LEFT_TOP_MOTOR = "motorLT";
    public static String RIGHT_TOP_MOTOR = "motorRT";
    public static String LEFT_BOTTOM_MOTOR = "motorLB";
    public static String RIGHT_BOTTOM_MOTOR = "motorRB";

    public static final class Etc {
        public static Telemetry telemetry;
    }



}
