package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Teleop: Blue", group="Teleop")
public class BlueTeleop extends OpMode implements Constants
{
    // Declare OpMode members.
    private ElapsedTime runtime;
    private RobotSystem robotSystem;
    private boolean autoAlign;
    private double desiredAngle;
    private double lastTurn;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Initialize Fields
        robotSystem = new RobotSystem();
        robotSystem.init(hardwareMap, IS_BLUE_ALLIANCE, 0.0, 0.0);
        autoAlign = true;
        desiredAngle = 90.0;
        lastTurn = 0.0;

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
         autoAlign = gamepad1.left_trigger == 0.0 && gamepad1.right_trigger == 0.0;

        double[] gamepadValues = robotSystem.getGamepadValues(gamepad1);
        double power = gamepadValues[0];
        double angle = gamepadValues[1];
        double turn = gamepadValues[2];
        double headingDegrees = robotSystem.getFieldHeading();

        if( lastTurn != 0.0 && turn == 0.0 )
            desiredAngle = robotSystem.getFieldHeading();

        if(gamepad1.y)
            desiredAngle = 90.0;
        else if(gamepad1.b)
            desiredAngle = 0.0;
        else if(gamepad1.a)
            desiredAngle = -90.0;
        else if(gamepad1.x)
            desiredAngle = -180.0;

        robotSystem.drive(power, angle, turn, autoAlign, desiredAngle);
        robotSystem.updateCoordinates();
        robotSystem.updateEncoderPositions();
        lastTurn = turn;

        // Show Telemetry
        double[] coordinates = robotSystem.getXY();
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Robot Field Heading", headingDegrees);
        telemetry.addData("X Coordinate", coordinates[0]);
        telemetry.addData("Y Coordinate", coordinates[1]);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {}
}