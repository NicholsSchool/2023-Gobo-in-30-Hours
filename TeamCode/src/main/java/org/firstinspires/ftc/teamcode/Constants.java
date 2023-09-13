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

    /** The max motor power used for driving */
    public static final double DRIVE_LIMIT = 0.75;

    /** The max motor power used for turning */
    public static final double TURN_LIMIT = .25;

    /** The maximum error deemed ok for auto alignment */
    public static final double ANGLE_THRESHOLD = .25;

    /** The Proportional constant used for auto alignment */
    public static final double TURN_P = .2;
}
