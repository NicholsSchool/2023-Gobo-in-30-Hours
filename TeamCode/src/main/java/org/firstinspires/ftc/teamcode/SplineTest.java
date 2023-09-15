package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Teleop: Spline Test", group="Teleop")
public class SplineTest extends OpMode implements Constants
{
    // Declare OpMode members.
    private ElapsedTime runtime;
    private RobotSystem robotSystem;
    private boolean autoAlign;
    private double desiredAngle;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Initialize Fields
        robotSystem = new RobotSystem();
        robotSystem.init(hardwareMap, IS_BLUE_ALLIANCE, 36.0, 12.0);
        autoAlign = true;
        desiredAngle = 90.0;

        // Initialize Runtime
        runtime = new ElapsedTime();

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {}

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double[] coordinates = robotSystem.getXY();

        double power = 0.5;
        double slope = (14 * Math.pow(Math.E, -.1 * (coordinates[0] - 86.0)))
                /(Math.pow(Math.pow(Math.E, -.1 * (coordinates[0] - 86.0)) + 1, 2) );

        if(coordinates[0] > 120)
            power = 0.0;

        double theta = Math.toDegrees(Math.atan(slope));

        robotSystem.drive(power, theta, 0.0, autoAlign, desiredAngle);

        robotSystem.updateCoordinates();
        robotSystem.updateEncoderPositions();

        // Show Telemetry
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("power", power);
        telemetry.addData("slope", slope);
        telemetry.addData("theta", theta);
        telemetry.addData("X Coordinate", coordinates[0]);
        telemetry.addData("Y Coordinate", coordinates[1]);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {}
}
