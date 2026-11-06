// language: java
package org.firstinspires.ftc.teamcode.PreviousYearsCode.DecodeOpModes.DecodeOpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Implementations.AndroidLog;
import org.firstinspires.ftc.teamcode.Implementations.SystemTimeSource;
import org.firstinspires.ftc.teamcode.Pathing.PID;
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


public class Decode extends LinearOpMode {

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
    public void










    runOpMode() throws InterruptedException {
        driveTrain = new DriveTrain(hardwareMap);
        imu = new PinpointImpl(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap, intake, log);
        PID pid = new PID(0.85, 0, 0);
        PID pidRotate = new PID(0.01, 0, 0);
        PowerRampController rampDrive = new PowerRampController(
                .2,
                new SystemTimeSource());


//        Through testing y's and x's are flipped
//        instead of:
//                y
//                |
//                |
//       ------------------x
//                |
//                |
//         we have:
//                x
//                |
//                |
//       ------------------y
//                |
//                |

        double y = 0;
        double x = 0;
        double h = 0;
        double targetDistance = 0;
        double mirrorField = 1;
        String team = "Blue Side";
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        imu.reset();

        double kp = 0.045;
        double ki = 0.0;
        double kd = 0.11;

        // these are for strafing
        double kps = 0.13;
        double kds = 0.29;


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
            //telemetry.addData("Add Delay, before robot starts?");
            //if(gamepad2.start){
            //sleep(5000);
            //  telemetry.addData("Delay Added")
            //} else {
            //    sleep(0);
            //}

            double change = 0.01;
            if (gamepad2.right_bumper) {
                change = .001;
            }

            if (gamepad2.a && gamepad2.dpad_up) {
                kds += .01;
            } else if (gamepad2.a && gamepad2.dpad_down) {
                kds -= .01;
            } else if (gamepad2.b && gamepad2.dpad_up) {
                ki += .00001;
            } else if (gamepad2.b && gamepad2.dpad_down) {
                ki -= .00001;
            } else if (gamepad2.y && gamepad2.dpad_up) {
                kps += .01;
            } else if (gamepad2.y && gamepad2.dpad_down) {
                kps -= .01;
            }

            telemetry.addData("PID Values:", "kp: %.3f ki: %.5f kd: %.3f", kps, ki, kds);

            telemetry.addData("Side", team);

            sleep(100);
            idle();
        }
//
        // Start autonomous
        stateTimer.reset();
        //sleep(5000);
        targetDistance = 52;

        telemetry.clearAll();
        telemetry.addData("Status", "Started");
        telemetry.addData("X Position", x);
        telemetry.addData("Y Position", y);
        telemetry.addData("Heading", h);
        telemetry.addData("Target", targetDistance);

        shooter.startShooterMotor(0.7);

        int count = 0;
        double startHeading = imu.getHeading(AngleUnit.DEGREES);


        try {
            // This comment is helpful
            shooter.startShooterMotor(0.6);
            driveTrain.driveStraight(-72, 0.8, imu, log, kp, ki, kd);
            driveTrain.rotateRelative(45 * mirrorField, imu, log);
            stateTimer.reset();
            while (shooter.getBallsLaunched() < 3 && opModeIsActive() && stateTimer.milliseconds() < 5000) {
                shooter.continousShoot(false, true, false);
                sleep(5);
            }
            shooter.resetShooter();
            sleep(1);
            driveTrain.rotateRelative(-135 * mirrorField, imu, log);
            shooter.startIntake();
            driveTrain.driveStraight(36, 0.4, imu, log, kp, ki, kd);
            shooter.resetShooter();
            sleep(1);
            driveTrain.driveStraight(-36, 0.8, imu, log, kp, ki, kd);
            driveTrain.rotateRelative(135 * mirrorField, imu, log);

            stateTimer.reset();
            while (shooter.getBallsLaunched() < 3 && opModeIsActive() && stateTimer.milliseconds() < 4000) {
                shooter.continousShoot(false, true, false);
                sleep(5);
            }
            shooter.resetShooter();
            driveTrain.rotateRelative(-135 * mirrorField, imu, log);
            driveTrain.driveStrafe(24 * mirrorField, 0.7, imu, log, kps, ki, kds);
            shooter.startIntake();
            driveTrain.driveStraight(40, 0.6, imu, log, kp, ki, kd);
            shooter.resetShooter();
            driveTrain.driveStraight(-4, 0.6, imu, log, kp, ki, kd);


        } catch (Exception e) {
            throw new RuntimeException(e);
        }
    }
}
