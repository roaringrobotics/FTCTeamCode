// language: java
package org.firstinspires.ftc.teamcode.PreviousYearsCode.DecodeOpModes.DecodeOpModes.Autonomous;

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


public class Leave extends LinearOpMode {

    private DriveTrain driveTrain;
    private Shooter shooter;
    private PinpointImpl imu;
    private Intake intake;
    private final AndroidLog log = new AndroidLog();


    // Hardware placeholders - change names to match your robot configuration
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    private final ElapsedTime stateTimer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        driveTrain = new DriveTrain(hardwareMap);
        imu = new PinpointImpl(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap, intake, log);

        PowerRampController rampDrive = new PowerRampController(
                .1,
                new SystemTimeSource());


/*     Through testing y's and x's are flipped
        instead of:
                y
                |
                |
       ------------------x
                |
                |
         we have:
                x
                |
                |
       ------------------y
                |
                |
*/

        double y = 0;
        double x = 0;
        double h = 0;
        double targetDistance = 0;
        double mirrorField = 1;
        String team = "Blue Side";
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        imu.reset();


        // show init telemetry until start pressed
        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Hint", "Waiting for start - update sensors/vision here");
            telemetry.update();
            if (gamepad2.optionsWasPressed()) {
                mirrorField = mirrorField * -1;
            }
            if (mirrorField == -1) {
                team = "Red Side";
            } else {
                team = "Blue Side";
            }
            telemetry.addData("Side", team);
            idle();
        }

        if (isStopRequested()) {
            return;
        }

        // Start autonomous
        stateTimer.reset();

        try {
            driveTrain.driveStraight(-4, 0.3, imu, log, 0.045, 0, 0);
            driveTrain.rotateRelative(-90 * mirrorField, imu, log);
            driveTrain.driveStraight(18, 0.4, imu, log, 0.045, 0, 0);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
    }
}