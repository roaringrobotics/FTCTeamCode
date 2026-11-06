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


public class DecodeShortwRedImp extends LinearOpMode {

    private DriveTrain driveTrain;
    private Shooter shooter;
    private PinpointImpl imu;
    private Intake intake;
    private final AndroidLog log = new AndroidLog();

    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    private final ElapsedTime stateTimer = new ElapsedTime();

    // Mirror + Override System
    double mirrorField = 1;
    boolean useRedOverrides = false;

    double allianceValue(double blueVal, double redVal) {
        if (mirrorField == -1 && useRedOverrides) {
            return redVal;   // true Red tuning
        }
        return blueVal * mirrorField; // default mirrored behavior
    }

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

        double y = 0;
        double x = 0;
        double h = 0;
        double targetDistance = 0;

        String team = "Blue Side";
        int rows = 0;
        boolean collectFromBasket = false;
        boolean stayInTriangle = false;

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        imu.reset();

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        // INIT LOOP
        while (!isStarted() && !isStopRequested()) {

            if (gamepad2.optionsWasPressed()) {
                mirrorField *= -1;  // switch alliance
            }

            if (gamepad2.xWasPressed()) {
                useRedOverrides = !useRedOverrides;
            }

            team = (mirrorField == -1) ? "Red Side" : "Blue Side";

            telemetry.addData("Side", team);
            telemetry.addData("Red Overrides", useRedOverrides);

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

            telemetry.update();
            idle();
        }

        if (isStopRequested()) return;

        double kp = 0.045;
        double ki = 0.0;
        double kd = 0.13;

        double kps = 0.13;
        double kds = 0.29;

        try {

            shooter.startShooterMotor(1350);

            driveTrain.driveStraight(-6, .8, imu, log, kp, ki, kd);

            driveTrain.rotateRelative(
                    allianceValue(23, -113),  // Blue / Red override
                    imu, log
            );

            stateTimer.reset();
            while (shooter.getBallsLaunched() < 3 &&
                    opModeIsActive() &&
                    stateTimer.milliseconds() < 4000) {

                shooter.continousShoot(1475);
            }

            shooter.resetShooter();

            for (int i = 1; i <= rows; i++) {

                driveTrain.rotateRelative(
                        allianceValue(-113, -108),
                        imu, log);

                driveTrain.driveStrafe(
                        allianceValue(-21 * i, -19 * i),
                        0.9, imu, log, kps, ki, kds);

                shooter.startIntake();

                driveTrain.driveStraight(42, 0.9, imu, log, kp, ki, kd);

                shooter.startShootServos();
                sleep(300);

                shooter.resetShooter();
                shooter.startShooterMotor((i > 1 ? 1100 : 1350));

                driveTrain.driveStraight(-42, 0.9, imu, log, kp, ki, kd);

                if (i <= 1 || stayInTriangle) {

                    driveTrain.driveStrafe(
                            allianceValue(21 * i, 19 * i),
                            0.9, imu, log, kps, ki, kds
                    );

                    driveTrain.rotateRelative(
                            allianceValue(113, 110),
                            imu, log
                    );

                } else {

                    driveTrain.driveStrafe(
                            allianceValue(-21, -19),
                            0.9, imu, log, kps, ki, kds
                    );

                    driveTrain.rotateRelative(
                            allianceValue(138, 135),
                            imu, log
                    );
                }

                stateTimer.reset();
                while (shooter.getBallsLaunched() < 3 &&
                        opModeIsActive() &&
                        stateTimer.milliseconds() < (i == 1 ? 4000 : 2750)) {

                    shooter.continousShoot((i > 1 ? 1150 : 1475));

                    dashboardTelemetry.addData("Velocity", shooter.getShooterVelocity());
                    dashboardTelemetry.update();
                }

                shooter.resetShooter();
            }

            if (collectFromBasket || rows <= 1) {

                if (rows <= 1 || stayInTriangle) {

                    driveTrain.rotateRelative(
                            allianceValue(157, 152),
                            imu, log
                    );

                    driveTrain.driveStrafe(
                            allianceValue(46, 44),
                            0.9, imu, log, kps, ki, kds
                    );

                } else {

                    driveTrain.rotateRelative(
                            allianceValue(133, 130),
                            imu, log
                    );

                    driveTrain.driveStraight(-44, 0.9, imu, log, kp, ki, kd);

                    driveTrain.driveStrafe(
                            allianceValue(49, 47),
                            0.9, imu, log, kps, ki, kds
                    );
                }

                shooter.startIntake();
                sleep(3000);

                shooter.startShootServos();
                shooter.startIntake();
                sleep(12000);

            } else {

                driveTrain.rotateRelative(
                        allianceValue(-140, -137),
                        imu, log
                );

                driveTrain.driveStrafe(
                        allianceValue(-38, -36),
                        0.9, imu, log, kps, ki, kds
                );
            }

        } catch (Exception e) {
            throw new RuntimeException(e);
        }
    }
}