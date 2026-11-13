package org.firstinspires.ftc.teamcode.OpModes.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Implementations.AndroidLog;
import org.firstinspires.ftc.teamcode.Implementations.SystemTimeSource;
import org.firstinspires.ftc.teamcode.Pathing.PowerRampController;
import org.firstinspires.ftc.teamcode.RobotHardware.DriveTrain;
import org.firstinspires.ftc.teamcode.RobotHardware.Intake;
import org.firstinspires.ftc.teamcode.RobotHardware.PinpointImpl;
import org.firstinspires.ftc.teamcode.RobotHardware.Shooter;


/*
  Autonomous skeleton:
  - simple state machine (INIT -> DRIVE -> TURN -> ACTION -> IDLE -> DONE)
  - hardware placeholders (left/right drive motors)
  - timed transitions using ElapsedTime
  - helper methods to set/stop drive power
*/
@Autonomous(group = "Test")
public class PIDControllerTesting extends LinearOpMode {

    private DriveTrain driveTrain;
    private PinpointImpl imu;
    private final AndroidLog log = new AndroidLog();


    // Hardware placeholders - change names to match your robot configuration
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    // autonomous 30s timer
    private final ElapsedTime stateTimer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        driveTrain = new DriveTrain(hardwareMap);
        imu = new PinpointImpl(hardwareMap);

        PowerRampController rampDrive = new PowerRampController(
                .1,
                new SystemTimeSource());
        // define PID controller starter values for PID testing
        double kpl = 0.3;
        double kil = 0;
        double kdl = 0;

        boolean isRed = false;


        double y = 0;
        double x = 0;
        double h = 0;
        double targetDistance = 0;
        double mirrorField = 1;
        String team;
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        imu.reset();

        double change = 0.01;

        // show init telemetry until start pressed
        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Hint", "Waiting for start - update sensors/vision here");
            telemetry.update();
            if (gamepad2.optionsWasPressed()) {
                isRed = !isRed;
            }
            if (isRed) {
                team = "Red Side";
            } else {
                team = "Blue Side";
            }


            // PID controller testing
            // used the defined variables
            if (gamepad2.rightBumperWasPressed()) {
                change *= 0.1;
            } else if (gamepad2.leftBumperWasPressed()) {
                change *= 10;
            }


            if (gamepad2.a && gamepad2.dpadUpWasReleased()) {
                kdl += change;
            } else if (gamepad2.a && gamepad2.dpadDownWasPressed()) {
                kdl -= change;
            } else if (gamepad2.b && gamepad2.dpadUpWasReleased()) {
                kil += change;
            } else if (gamepad2.b && gamepad2.dpadDownWasPressed()) {
                kil -= change;
            } else if (gamepad2.y && gamepad2.dpadUpWasReleased()) {
                kpl += change;
            } else if (gamepad2.y && gamepad2.dpadDownWasPressed()) {
                kpl -= change;
            }



            telemetry.addData("PID Values", "kp: %.5f ki: %.5f kd: %.5f", kpl, kil, kdl);
            telemetry.addData("PID Change", change);
            telemetry.addData("Side", team);
            idle();
        }

        if (isStopRequested()) {
            return;
        }

        // Start autonomous
        stateTimer.reset();
        try {
            // write code here for autonomous
            driveTrain.driveToRelative(12, 12, 0, 1.0, isRed, imu, kpl, kil, kdl, 1, 0, 0);


            // write code here for autonomous
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
    }
}
