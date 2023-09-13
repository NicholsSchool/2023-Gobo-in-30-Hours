package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Teleop: Tune Feedforward", group="Teleop")
public class FeedForwardTeleop extends OpMode implements Constants
{
    // Declare OpMode members.
    private ElapsedTime runtime;
    private RobotSystem robotSystem;
    private String motor;
    private double increment;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Initialize Fields
        robotSystem = new RobotSystem();
        robotSystem.init(hardwareMap, IS_BLUE_ALLIANCE);
        motor = "frontRight";
        increment = 0.1;

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
        if( gamepad1.dpad_up && runtime.time() > 0.25 )
        {
            motor = "frontRight";
            runtime.reset();
        }
        else if( gamepad1.dpad_right && runtime.time() > 0.25 )
        {
            motor = "backRight";
            runtime.reset();
        }
        else if( gamepad1.dpad_down && runtime.time() > 0.25 )
        {
            motor = "backLeft";
            runtime.reset();
        }
        else if( gamepad1.dpad_left && runtime.time() > 0.25 )
        {
            motor = "frontLeft";
            runtime.reset();
        }
        else if( gamepad1.y && runtime.time() > 0.25 )
        {
            increment *= 10;
            runtime.reset();
        }
        else if( gamepad1.a && runtime.time() > 0.25 )
        {
            increment /= 10;
            runtime.reset();
        }
        else if( gamepad1.b && runtime.time() > 0.25 )
        {
            switch (motor) {
                case "backRight":
                    robotSystem.backRight.setVelocityPIDFCoefficients( 0, 0, 0,
                            robotSystem.backRight.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).f + increment);
                    break;
                case "backLeft":
                    robotSystem.backLeft.setVelocityPIDFCoefficients( 0, 0, 0,
                            robotSystem.backLeft.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).f + increment);
                    break;
                case "frontRight":
                    robotSystem.frontRight.setVelocityPIDFCoefficients( 0, 0, 0,
                            robotSystem.frontRight.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).f + increment);
                    break;
                case "frontLeft":
                    robotSystem.frontLeft.setVelocityPIDFCoefficients( 0, 0, 0,
                            robotSystem.frontLeft.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).f + increment);
                    break;
            }
            runtime.reset();
        }
        else if( gamepad1.x && runtime.time() > 0.25 )
        {
            increment *= -1;
            runtime.reset();
        }
        else if( gamepad1.back && runtime.time() > 0.25 )
        {
            increment = 0.1;
            runtime.reset();
        }


        if( gamepad1.left_trigger > 0.1 )
        {
            robotSystem.driveTest(.75);
        }
        else if( gamepad1.right_trigger > 0.1 )
        {
            robotSystem.driveTest(-.75);
        }
        else
            robotSystem.driveTest(0.0);

        // Show Telemetry
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Increment", increment);
        telemetry.addData("Which motor", motor);
        telemetry.addData("backLeft F", robotSystem.backLeft.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).f);
        telemetry.addData("backRight F", robotSystem.backRight.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).f);
        telemetry.addData("frontLeft F", robotSystem.frontLeft.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).f);
        telemetry.addData("frontRight F", robotSystem.frontRight.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).f);
        double[] motorSpeeds = robotSystem.getMotorVelocities();
        telemetry.addData("BackLeft Vel", motorSpeeds[0]);
        telemetry.addData("BackRight Vel", motorSpeeds[1]);
        telemetry.addData("FrontLeft Vel", motorSpeeds[2]);
        telemetry.addData("FrontRight Vel", motorSpeeds[3]);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {}
}