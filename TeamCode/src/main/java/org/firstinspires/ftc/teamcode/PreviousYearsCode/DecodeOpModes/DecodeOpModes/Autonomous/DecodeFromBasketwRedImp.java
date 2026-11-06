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


public class DecodeFromBasketwRedImp extends LinearOpMode {

    private DriveTrain driveTrain;
    private Shooter shooter;
    private PinpointImpl imu;
    private Intake intake;
    private final AndroidLog log = new AndroidLog();

    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    private final ElapsedTime stateTimer = new ElapsedTime();

    // ✅ Mirror + Override System
    double mirrorField = 1;
    boolean useRedOverrides = false;

    double allianceValue(double blueVal, double redVal) {
        if (mirrorField == -1 && useRedOverrides) {
            return redVal;
        }
        return blueVal * mirrorField;
    }

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

        double y = 0;
        double x = 0;
        double h = 0;

        String team;
        int rows = 0;

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        imu.reset();

        // ✅ INIT LOOP
        while (!isStarted() && !isStopRequested()) {

            if (gamepad2.optionsWasPressed()) {
                mirrorField *= -1;
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

            rows = Math.abs(rows) % 3;

            telemetry.addData("Rows", rows);
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

            shooter.startShooterMotor(1000);

            driveTrain.driveStraight(55, 0.9, imu, log, kp, ki, kd);

            stateTimer.reset();
            while (shooter.getBallsLaunched() < 3 &&
                    opModeIsActive() &&
                    stateTimer.milliseconds() < 2750) {

                shooter.continousShoot(1160);
            }

            shooter.resetShooter();
            sleep(1);

            for (int i = 0; i < rows; i++) {

                if (i == 0) {
                    driveTrain.driveStrafe(
                            allianceValue(-11, -9), 0.9, imu, log, kps, ki, kds
                    );
                }

                driveTrain.rotateRelative(
                        allianceValue(
                                (i == 0 ? -140 : -132),
                                (i == 0 ? -136 : -128)
                        ),
                        imu, log
                );

                driveTrain.driveStrafe(
                        allianceValue(25 * i, 23 * i), 0.9, imu, log, kps, ki, kds
                );

                shooter.startIntake();

                driveTrain.driveStraight(
                        allianceValue(40 + i * 5, 38 + i * 5), 0.9, imu, log, kp, ki, kd
                );

                shooter.startShootServos();
                sleep(300);

                shooter.resetShooter();

                driveTrain.driveStraight(
                        allianceValue(-40 + i * 5, -38 + i * 5), 0.9, imu, log, kp, ki, kd
                );

                driveTrain.driveStrafe(
                        allianceValue(-25 * i, -23 * i), 0.9, imu, log, kps, ki, kds
                );

                driveTrain.rotateRelative(
                        allianceValue(
                                (i == 1 ? 138 : 132),
                                (i == 1 ? 134 : 128)
                        ), imu, log
                );

                stateTimer.reset();
                while (shooter.getBallsLaunched() < 3 &&
                        opModeIsActive() && stateTimer.milliseconds() < 2750 - i * 100) {

                    shooter.continousShoot(
                            (int) allianceValue(
                                    (i == 0 ? 1050 : 1100),
                                    (i == 0 ? 1020 : 1080)
                            )
                    );
                }

                shooter.resetShooter();
            }

            driveTrain.rotateRelative(
                    allianceValue(-50, -47), imu, log
            );

            driveTrain.driveStraight(-38, 0.9, imu, log, kps, ki, kds);

            driveTrain.rotateRelative(-90, imu, log);

        } catch (Exception e) {
            throw new RuntimeException(e);
        }
    }
}