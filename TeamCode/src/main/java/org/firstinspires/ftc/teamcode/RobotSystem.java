package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * The Class defining the Robot system
 */
public class RobotSystem implements Constants {
    private BHI260IMU imu;
    DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private boolean isBlueAlliance;

    public void init(HardwareMap hwMap, boolean isBlueAlliance)
    {
        // Initialize Variables
        this.isBlueAlliance = isBlueAlliance;

        // Instantiating IMU Parameters, setting angleUnit...
        BHI260IMU.Parameters params = new BHI260IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );

        // Defining and Initializing IMU... Initializing it with the above Parameters...
        imu = hwMap.get(BHI260IMU.class, "the_imu");
        imu.initialize(params);
        imu.resetYaw(); //Don't do this for actual matches

        // Initialize Motors
        backLeft = hwMap.get(DcMotorEx.class, "backLeft");
        backRight = hwMap.get(DcMotorEx.class, "backRight");
        frontLeft = hwMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hwMap.get(DcMotorEx.class, "frontRight");

        // Invert Motors
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set Zero Power Behavior
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Stop and Reset Encoders
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set Motor RunModes
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set Motor FF Coefficients
        backLeft.setVelocityPIDFCoefficients(0, 0, 0, BACK_LEFT_FF);
        backRight.setVelocityPIDFCoefficients(0, 0, 0, BACK_RIGHT_FF);
        frontLeft.setVelocityPIDFCoefficients(0, 0, 0, FRONT_LEFT_FF);
        frontRight.setVelocityPIDFCoefficients(0, 0, 0, FRONT_RIGHT_FF);
    }

    /**
     * A method used to tune FeedForward Values, spins motors at equal power
     *
     * @param power the power to spin motors at
     */
    public void driveTest(double power)
    {
        backLeft.setPower(power);
        backRight.setPower(power);
        frontLeft.setPower(power);
        frontRight.setPower(power);
    }

    /**
     * Drives the robot field-oriented
     *
     * @param power the driving power
     * @param angle the angle to drive at in degrees
     * @param turn the turning power
     */
    public void drive(double power, double angle, double turn, boolean autoAlign, double desiredAngle)
    {
        power = Range.clip(power, -DRIVE_LIMIT, DRIVE_LIMIT);
        turn = Range.clip(turn, -TURN_LIMIT, TURN_LIMIT);

        double heading = this.getFieldHeading();
        if(autoAlign)
            turn = this.autoAlign(desiredAngle, heading);

        double corner1 = power * Math.sin(Math.toRadians(angle - 45.0 + 90 - heading));
        double corner2 = power * Math.sin(Math.toRadians(angle + 45.0 + 90 - heading));

        backLeft.setPower(corner1 + turn);
        backRight.setPower(corner2 - turn);
        frontLeft.setPower(corner2 + turn);
        frontRight.setPower(corner1 - turn);
    }

    /**
     * Corrects Robot Heading
     *
     * @return the turn speed to replace the chosen turn speed
     */
    public double autoAlign(double desiredAngle, double heading) {
        double difference = desiredAngle - heading;
        if (difference < -180.0)
            difference += 360.0;
        else if (difference >= 180.0)
            difference -= 360.0;

        if (Math.abs(difference) < ANGLE_THRESHOLD)
            return 0.0;
        else
            return Range.clip(difference * TURN_P, -TURN_LIMIT, TURN_LIMIT);
    }

    /**
     * Gets the Gamepad Values used in Teleop
     *
     * @param gamepad the gamepad to take input from
     *
     * @return the drive power [-1, 1], angle [-180, 180), turn power [-1, 1]
     */
    public double[] getGamepadValues(Gamepad gamepad)
    {
        double x = gamepad.left_stick_x;
        double y = -gamepad.left_stick_y;
        double power = Range.clip(Math.sqrt(x * x + y * y), 0.0, 1.0);
        double angle = Math.toDegrees(Math.atan2(y, x));
        if(!isBlueAlliance)
        {
            angle += 180.0;
            if(angle >= 180.0)
                angle -= 360;
        }
        double turn = gamepad.right_trigger - gamepad.left_trigger;

        return new double[]{power, angle, turn};
    }

    /**
     * Gets the heading of the robot on the field coordinate system
     *
     * @return the heading in degrees [-180, 180)
     */
    public double getFieldHeading()
    {
        double angle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        if (isBlueAlliance)
        {
            if( angle >= 90.0 )
                return angle - 270.0;
            else
                return angle + 90.0;
        }
        else
        {
            if( angle <= -90.0 )
                return angle + 270.0;
            else
                return angle - 90.0;
        }
    }

    /**
     * Get Motor Velocities
     *
     * @return in order: backLeft, backRight, frontLeft, frontRight speeds
     */
    public double[] getMotorVelocities()
    {
        return new double[]{
                backLeft.getVelocity(),
                backRight.getVelocity(),
                frontLeft.getVelocity(),
                frontRight.getVelocity()
        };
    }
}