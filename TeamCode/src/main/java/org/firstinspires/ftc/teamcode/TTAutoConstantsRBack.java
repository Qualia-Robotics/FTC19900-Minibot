package org.firstinspires.ftc.teamcode;

/**
 * Constants for TechnoTacos Autonomous Mode
 * 
 * All autonomous movement and mechanism parameters are defined here.
 * Change values in this file to tune autonomous behavior across all OpModes.
 */
public class TTAutoConstantsRBack {

    // ==================== MOVEMENT DISTANCES ====================

    /** Distance to back up from starting position to shooting position (inches) */
    public static final double DRIVE_TO_SHOOT = -84.0;

    /** Distance to drive forward to collect balls (inches) */
    public static final double DRIVE_TO_COLLECT = -32.0; // this may need to be negative

    /** Strafe distance for first ball group (inches) */
    public static final double STRAFE_GROUP_1 = -13.2;

    /** Strafe distance for second ball group (inches) */
    public static final double STRAFE_GROUP_2 = -38.5;


    /** Strafe distance for third ball group (inches) */
    public static final double STRAFE_GROUP_3 = -60.5;

    /** Array of all strafe distances for easy iteration */
    public static final double[] STRAFE_DISTANCES = {
            STRAFE_GROUP_1,
            STRAFE_GROUP_2,
            STRAFE_GROUP_3
    };
    public static final double STRAFE_LEAVE = -3;
    public static final double COLLECT_GROUP_1 = -32;
    public static final double COLLECT_GROUP_2 = -38;
    public static final double COLLECT_GROUP_3 = -38;
    public static final double[] COLLECT_DISTANCES = {
            COLLECT_GROUP_1,
            COLLECT_GROUP_2,
            COLLECT_GROUP_3
    };
    // ==================== ROTATION ANGLES ====================

    /** Starting heading angle (degrees) */
    public static final double START_HEADING = 45.0;

    /** Rotation angle before collecting balls - counterclockwise (degrees) */
    public static final double ROTATE_TO_COLLECT = 135.0;

    /** Rotation angle after collecting balls - clockwise (degrees) */
    public static final double ROTATE_TO_SHOOT = -135.0;

    public static final double ROTATE_TO_LEAVE = -45.0;

    // ==================== MOTOR POWERS ====================

    /** Drive motor power during autonomous movements (0.0 to 1.0) */
    public static final double DRIVE_POWER = 1;
    public static final double DRIVE_POWER_COLLECT = 0.4;

    /** Rotation power during turns (0.0 to 1.0) */
    public static final double ROTATE_POWER = 0.9;

    /** Left launcher motor power during shooting */
    public static final double LEFT_LAUNCH_POWER = 0.83;

    /** Right launcher motor power during shooting (opposite direction) */
    public static final double RIGHT_LAUNCH_POWER = -0.83;

    /** Index motor power during shooting (feeding balls) */
    public static final double INDEX_SHOOT_POWER = -0.9;

    public static final double INDEX_SHOOT_POWER2 = -0.95;
    public static final double INDEX_DROP = 1;
    /** Index motor power during collection (intake) */
    public static final double INDEX_COLLECT_POWER = -1.0;

    /** Rotation servo power during shooting */
    public static final double ROTATE_SERVO_POWER = -1.0;

    /** Second rotation servo power during shooting (opposite direction) */
    public static final double ROTATE2_SERVO_POWER = 1.0;

    // ==================== TIMING VALUES ====================

    /** Time to spin up launchers before shooting (milliseconds) */
    public static final long LAUNCHER_SPINUP_TIME = 200;

    /** Time to run shooting sequence (milliseconds) */
    public static final long SHOOTING_DURATION1 = 1900;
    public static final long SHOOTING_DURATION2 = 300;

    // ==================== TOLERANCE VALUES ====================

    /** Rotation tolerance for odometry turnTo command (degrees) */
    public static final double ROTATION_TOLERANCE = 6.0;

    // Private constructor prevents instantiation
    private TTAutoConstantsRBack() {
        throw new AssertionError("Constants class should not be instantiated");
    }
}
