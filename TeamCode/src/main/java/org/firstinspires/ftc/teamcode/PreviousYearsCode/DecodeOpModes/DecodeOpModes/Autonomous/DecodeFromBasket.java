package org.firstinspires.ftc.teamcode.PreviousYearsCode.DecodeOpModes.DecodeOpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

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


public class DecodeFromBasket extends LinearOpMode {

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
        PID pid = new PID(0.8, 0, 0);
        PID pidRotate = new PID(0.02, 0, 0);
        PowerRampController rampDrive = new PowerRampController(
                .1,
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
        double mirrorField = 1;
        String team;
        int rows = 0;
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
            if (gamepad2.dpadUpWasPressed()) {
                rows++;
            } else if (gamepad2.dpadDownWasPressed()) {
                rows--;
            }
            rows = Math.abs(rows) % 3;
            telemetry.addData("Rows", rows);
        }

        if (isStopRequested()) {
            return;
        }
        double kp = 0.045;
        double ki = 0.0;
        double kd = 0.13;
        // Start autonomous

        // these are for strafing
        double kps = 0.13;
        double kds = 0.29;
        try {
            shooter.startShooterMotor(1000);
            driveTrain.driveStraight(55, 0.9, imu, log, kp, ki, kd);
//            driveTrain.rotateRelative(-138 * mirrorField, imu, log);

            stateTimer.reset();

            while (shooter.getBallsLaunched() < 3 && opModeIsActive() && stateTimer.milliseconds() < 2750) {
                shooter.continousShoot(1160);
            }
            shooter.resetShooter();
            sleep(1);
            for (int i = 0; i < rows; i++) {



                if (i == 0) {
                    driveTrain.driveStrafe(-11 * mirrorField, 0.9, imu, log, kps, ki, kds);
                }
                driveTrain.rotateRelative((i == 0 ? -140 : -132) * mirrorField, imu, log);
                driveTrain.driveStrafe(25 * i * mirrorField, 0.9, imu, log, kps, ki, kds);
                shooter.startIntake();
                driveTrain.driveStraight(40 + i * 5, 0.9, imu, log, kp, ki, kd);

                // keep the intake running for a bit to ensure we get the balls in
                shooter.startShootServos();
                sleep(300);
                shooter.resetShooter();
                driveTrain.driveStraight(-40 + i * 5, 0.9, imu, log, kp, ki, kd);
                driveTrain.driveStrafe(-25 * i * mirrorField, 0.9, imu, log, kps, ki, kds);
                driveTrain.rotateRelative((i == 1 ? 138 : 132) * mirrorField, imu, log);

                stateTimer.reset();
                while (shooter.getBallsLaunched() < 3 && opModeIsActive() && stateTimer.milliseconds() < 2750 - i * 100) {
                    shooter.continousShoot((i == 0 ? 1050 : 1100));
                }
                shooter.resetShooter();
            }
            driveTrain.rotateRelative(-50 * mirrorField, imu, log);
            driveTrain.driveStraight(-38, 0.9, imu, log, kps, ki, kds);
            driveTrain.rotateRelative(-90, imu, log);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
    }
}