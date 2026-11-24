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

public class AutonomousSkeleton extends LinearOpMode {

    /*
    define hardware we need here

    Exp. for dashboard
    ftcDashboard log
        private FtcDashboard dashboard = FtcDashboard.getInstance();
     */
    private DriveTrain driveTrain;

    // autonomous 30s timer
    private final ElapsedTime stateTimer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        /*
        initialize hardware
        initialize all hardware or else you will get a null pointer error
         */
        driveTrain = new DriveTrain(hardwareMap);

        // isRed used to switch autonomous to different side
        boolean isRed = false;
        String team;

        // if using an imu always reset the imu before use
        // imu.reset();

        // show init telemetry until start pressed
        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Status", "Initialized");

            // logic for switching team colors
            if (gamepad2.optionsWasPressed()) {
                isRed = !isRed;
            }
            if (isRed) {
                team = "Red Side";
            } else {
                team = "Blue Side";
            }

            // add any more telemetry here
            telemetry.addData("Side", team);
            telemetry.update();
            idle();
        }

        stateTimer.reset();
        telemetry.addData("Status", "Started");
        telemetry.update();
        // Start autonomous



    }
}
