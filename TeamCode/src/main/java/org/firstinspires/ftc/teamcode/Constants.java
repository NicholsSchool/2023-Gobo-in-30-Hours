package org.firstinspires.ftc.teamcode;

/**
 * Drive Constants for the Robot
 */
interface Constants {
    /** Whether we are blue alliance */
    public static final boolean IS_BLUE_ALLIANCE = true;

    /** The Back Left FeedForward Constant */
    public static final double BACK_LEFT_FF = 12.3;

    /** The Back Right FeedForward Constant */
    public static final double BACK_RIGHT_FF = 12.5;

    /** The Front Left FeedForward Constant */
    public static final double FRONT_LEFT_FF = 13.0;

    /** The Front Right FeedForward Constant */
    public static final double FRONT_RIGHT_FF = 13.3;

    /** The max motor power applied to any drive motor */
    public static final double MAX_LIMIT = 1.0;

    /** The max motor power used for turning */
    public static final double TURN_LIMIT = .25;

    /** The maximum error deemed ok for auto alignment */
    public static final double ANGLE_THRESHOLD = 0.5;

    /** The Proportional constant used for auto alignment */
    public static final double TURN_P = .019;

    /** Ticks per revolution of the dead wheels */
    public static final int TICKS_PER_REV = 8192;

    /** Approximate Diameter of our Dead Wheels */
    public static final double DEAD_DIAMETER = 2.5;

    /** Inches driven per encoder tick of a dead wheel */
    public static final double INCHES_PER_TICK = DEAD_DIAMETER * Math.PI / TICKS_PER_REV;

    /** The Multiplier to forward distance tracking */
    public static final double FORWARD_ODOMETRY_CORRECTION = 1.02696500287;

    /** The Multiplier to strafe distance tracking */
    public static final double STRAFE_ODOMETRY_CORRECTION = 1.15260785576;

    /** The Multiplier the slope correction for splining */
    public static final double SPLINE_CORRECT = 0.05;
}
