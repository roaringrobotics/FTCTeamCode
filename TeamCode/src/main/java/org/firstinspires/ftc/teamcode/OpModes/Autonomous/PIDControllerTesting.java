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
        double kpl = 0.08;
        double kil = 0;
        double kdl = 2.5;
        // PID controller values, for rotation
        double kpr = 1;
        double kir = 0;
        double kdr = 0;

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

            // linear PID value changing during initialization
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
            // roatation PID value changing during initialization
            if (gamepad2.a && gamepad2.dpadRightWasReleased()) {
                kdr += change;
            } else if (gamepad2.a && gamepad2.dpadLeftWasPressed()) {
                kdr -= change;
            } else if (gamepad2.b && gamepad2.dpadRightWasReleased()) {
                kir += change;
            } else if (gamepad2.b && gamepad2.dpadLeftWasPressed()) {
                kir -= change;
            } else if (gamepad2.y && gamepad2.dpadRightWasReleased()) {
                kpr += change;
            } else if (gamepad2.y && gamepad2.dpadLeftWasPressed()) {
                kpr -= change;
            }
    // Notes for PID controller tuning
            // Left bumper updates change value, to 0.1. Right bumper changes it back
            // Up on D-pad increases linear PID values, down on D-pad decreases linear PID value
            // Right on D-pad increases roatation PID values, left on D-pad decreases rotation PID values
            // "Y / up" on controller sets value to change as, P
            // "B / right" on controller sets value to change as, I
            // "A / down" on controller sets value to change as, D

            telemetry.addData("Linear PID Values", "kp: %.5f ki: %.5f kd: %.5f", kpl, kil, kdl);
            telemetry.addData("Rotation PID Values", "kp: %.5f ki: %.5f kd: %.5f", kpr, kir, kdr);
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
            driveTrain.driveToRelative(12, 12, 0, 1.0, isRed, imu, kpl, kil, kdl, kpr, kir, kdr);


            // write code here for autonomous
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
    }
}
