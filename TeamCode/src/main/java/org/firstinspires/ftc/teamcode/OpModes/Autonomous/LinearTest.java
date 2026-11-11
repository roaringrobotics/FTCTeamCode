// language: java
package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Implementations.AndroidLog;
import org.firstinspires.ftc.teamcode.Implementations.SystemTimeSource;
import org.firstinspires.ftc.teamcode.Pathing.PowerRampController;
import org.firstinspires.ftc.teamcode.RobotHardware.DriveTrain;
import org.firstinspires.ftc.teamcode.RobotHardware.PinpointImpl;


/*
  Autonomous skeleton:
  - simple state machine (INIT -> DRIVE -> TURN -> ACTION -> IDLE -> DONE)
  - hardware placeholders (left/right drive motors)
  - timed transitions using ElapsedTime
  - helper methods to set/stop drive power
*/

@Autonomous(name = "Linear Test", group = "Test")
public class LinearTest extends LinearOpMode {

    private DriveTrain driveTrain;
    private PinpointImpl imu;
    private final AndroidLog log = new AndroidLog();
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();


    // Hardware placeholders - change names to match your robot configuration
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    private final ElapsedTime stateTimer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        driveTrain = new DriveTrain(hardwareMap);
        imu = new PinpointImpl(hardwareMap);

        PowerRampController rampDrive = new PowerRampController(
                .1,
                new SystemTimeSource());
        double kp = 1.0;
        double ki = 0;
        double kd = 0;

        boolean isRed = false;

        double y = 0;
        double x = 0;
        double h = 0;
        double targetDistance = 0;
        double mirrorField = 1;
        double power = 0.5;
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

            if (gamepad2.rightBumperWasPressed()) {
                change *= 0.1;
            } else if (gamepad2.leftBumperWasPressed()) {
                change *= 10;
            }


            if (gamepad2.a && gamepad2.dpadUpWasReleased()) {
                kd += change;
            } else if (gamepad2.a && gamepad2.dpadDownWasPressed()) {
                kd -= change;
            } else if (gamepad2.b && gamepad2.dpadUpWasReleased()) {
                ki += change;
            } else if (gamepad2.b && gamepad2.dpadDownWasPressed()) {
                ki -= change;
            } else if (gamepad2.y && gamepad2.dpadUpWasReleased()) {
                kp += change;
            } else if (gamepad2.y && gamepad2.dpadDownWasPressed()) {
                kp -= change;
            }



            telemetry.addData("PID Values", "kp: %.5f ki: %.5f kd: %.5f", kp, ki, kd);
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
            driveTrain.driveToFieldCentric(-12, -12, 45, power, imu);
            driveTrain.driveToFieldCentric(12, -12, -45, power, imu);
            driveTrain.driveToFieldCentric(12, 12, 45, power, imu);
            driveTrain.driveToFieldCentric(-12, 12, -45, power, imu);
            driveTrain.driveToFieldCentric(0, 0, 0, power, imu);

            dashboardTelemetry.addData("Done!", "");
            dashboardTelemetry.update();
            driveTrain.stopMotors();
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
    }
}