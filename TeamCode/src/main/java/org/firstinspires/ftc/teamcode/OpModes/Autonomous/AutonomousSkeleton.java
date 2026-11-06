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
        example:
        private DriveTrain driveTrain;

    android log
        private final AndroidLog log = new AndroidLog();

    ftcDashboard log
        private FtcDashboard dashboard = FtcDashboard.getInstance();
     */
    private DriveTrain driveTrain;
    private PinpointImpl imu;


    // Hardware placeholders - change names to match your robot configuration
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    // autonomous 30s timer
    private final ElapsedTime stateTimer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        /*
        initialize hardware
            example:
            driveTrain = new DriveTrain(hardwareMap);
        initialize all hardware or else you will get a null pointer error
         */
        driveTrain = new DriveTrain(hardwareMap);
        imu = new PinpointImpl(hardwareMap);

        // isRed used to switch autonomous to different side
        boolean isRed = false;
        String team;

        // placeholder values
        double y = 0;
        double x = 0;
        double h = 0;
        double targetDistance = 0;
        double mirrorField = 1;

        // if using an imu always reset the imu before use
        imu.reset();

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
            telemetry.addData("Side", team);
            telemetry.update();
            idle();
        }

        if (isStopRequested()) {
            return;
        }
        stateTimer.reset();

        // Start autonomous
        try {
            // write code here for autonomous



            // write code here for autonomous
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
    }
}
