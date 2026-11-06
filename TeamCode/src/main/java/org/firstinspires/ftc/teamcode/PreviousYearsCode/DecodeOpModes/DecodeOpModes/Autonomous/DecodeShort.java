// language: java
package org.firstinspires.ftc.teamcode.PreviousYearsCode.DecodeOpModes.DecodeOpModes.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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



public class DecodeShort extends LinearOpMode {

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
        PID pid = new PID(0.5, 0, 0);
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
        double targetDistance = 0;

        double mirrorField = 1;
        String team = "Blue Side";
        int rows = 0;
        boolean collectFromBasket = false;
        boolean stayInTriangle = false;
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        imu.reset();

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();
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
            rows = Math.abs(rows) % 4;
            telemetry.addData("Rows", rows);

            if (gamepad2.shareWasPressed()) {
                collectFromBasket = !collectFromBasket;
            }
            telemetry.addData("Collect From Basket", collectFromBasket);

            if (gamepad2.aWasPressed()) {
                stayInTriangle = !stayInTriangle;
            }
            telemetry.addData("Stay in Triangle", stayInTriangle);
            idle();
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

        // Start autonomous
        try {
            shooter.startShooterMotor(1350);
            driveTrain.driveStraight(-6, .8, imu, log, kp, ki, kd);
            driveTrain.rotateRelative(23 * mirrorField, imu, log);
            stateTimer.reset();
            while (shooter.getBallsLaunched() < 3 && opModeIsActive() && stateTimer.milliseconds() < 4000) {
                shooter.continousShoot(1475);
            }
            shooter.resetShooter();
            for (int i = 1; i <= rows; i++) {
                driveTrain.rotateRelative(-113 * mirrorField, imu, log);
                driveTrain.driveStrafe(-21 * i * mirrorField, 0.9, imu, log, kps, ki, kds);
                shooter.startIntake();
                driveTrain.driveStraight(42, 0.9, imu, log, kp, ki, kd);

                shooter.startShootServos();
                sleep(300);
                shooter.resetShooter();
                shooter.startShooterMotor((i > 1 ? 1100 : 1350));
                driveTrain.driveStraight(-42, 0.9, imu, log, kp, ki, kd);
                if (i <= 1 || stayInTriangle) {
                    driveTrain.driveStrafe(21 * i * mirrorField, 0.9, imu, log, kps, ki, kds);
                    driveTrain.rotateRelative(113 * mirrorField, imu, log);
                } else {
                    driveTrain.driveStrafe(-21 * mirrorField, 0.9, imu, log, kps, ki, kds);
                    driveTrain.rotateRelative(138 * mirrorField, imu, log);
                }

                stateTimer.reset();
                while (shooter.getBallsLaunched() < 3 && opModeIsActive() && stateTimer.milliseconds() < (i == 1 ? 4000 : 2750)) {
                    shooter.continousShoot((i > 1 ? 1150 : 1475));
                    dashboardTelemetry.addData("Velocity", shooter.getShooterVelocity());
                    dashboardTelemetry.update();
                }
                shooter.resetShooter();

            }
            if (collectFromBasket || rows <= 1) {
                if (rows <= 1 || stayInTriangle) {
                    driveTrain.rotateRelative(157 * mirrorField, imu, log);
                    driveTrain.driveStrafe(46 * mirrorField, 0.9, imu, log, kps, ki, kds);

                } else {
                    driveTrain.rotateRelative(133 * mirrorField, imu, log);
                    driveTrain.driveStraight(-44, 0.9, imu, log, kp, ki, kd);
                    driveTrain.driveStrafe(49 * mirrorField, 0.9, imu, log, kps, ki, kds);
                }
//                driveTrain.rotateRelative(-90 * mirrorField, imu, log);
                shooter.startIntake();

                // wait for balls to start to be intaken
                sleep(3000);
                shooter.startShootServos();
                shooter.startIntake();
                // sleep until auto is over
                sleep(12000);
            } else {
                driveTrain.rotateRelative(-140 * mirrorField, imu, log);
                driveTrain.driveStrafe(-38 * mirrorField, 0.9, imu, log, kps, ki, kds);
            }

    }   catch (Exception e) {
            throw new RuntimeException(e);
        }












        }
    }









