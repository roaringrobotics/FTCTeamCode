package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Tests;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Implementations.AndroidLog;
import org.firstinspires.ftc.teamcode.Implementations.SystemTimeSource;
import org.firstinspires.ftc.teamcode.Pathing.PID;
import org.firstinspires.ftc.teamcode.Pathing.PowerRampController;
import org.firstinspires.ftc.teamcode.RobotHardware.Intake;

import com.acmerobotics.dashboard.FtcDashboard;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.RobotHardware.Shooter;

@Autonomous(name = "Shooter Motor", group = "Test")
public class ShooterMotorTest extends LinearOpMode {
    private Shooter shooter;
    private final AndroidLog log = new AndroidLog();

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    @Override
    public void runOpMode() {
        shooter = new Shooter(hardwareMap, log);

        PID pid = new PID(0.5, 0, 0);
        PID pidRotate = new PID(0.02, 0, 0);
        PowerRampController rampDrive = new PowerRampController(
                .1,
                new SystemTimeSource());

        double y = 0;
        double x = 0;
        double h = 0;

        // These values work for shooter motor
        double kps = 350.0;
        double kis = 1.0;
        double kds = 1.0;

        // I don't get how these values are so different
        double kpi = 1.0;
        double kii = 2.0;
        double kdi = 5.0;
        double change = 1;

        double targetDistance = 0;


        // show init telemetry until start pressed
        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Hint", "Waiting for start - update sensors/vision here");

            // Changing PID values on driver hub
            if (gamepad2.rightBumperWasPressed()) {
                change *= 0.1;
            } else if (gamepad2.leftBumperWasPressed()) {
                change *= 10;
            }

            if (gamepad2.a && gamepad2.dpadUpWasReleased()) {
                kdi += change;
            } else if (gamepad2.a && gamepad2.dpadDownWasPressed()) {
                kdi -= change;
            } else if (gamepad2.b && gamepad2.dpadUpWasReleased()) {
                kii += change;
            } else if (gamepad2.b && gamepad2.dpadDownWasPressed()) {
                kii -= change;
            } else if (gamepad2.y && gamepad2.dpadUpWasReleased()) {
                kpi += change;
            } else if (gamepad2.y && gamepad2.dpadDownWasPressed()) {
                kpi -= change;
            }

            telemetry.addData("PID Values", "kp: %.5f ki: %.5f kd: %.5f", kpi, kii, kdi);
            telemetry.addData("PID Change", change);
            telemetry.update();
            shooter.setPidShooterValues(kps, kis, kds);
            shooter.setPidIntakeValues(kpi, kii, kdi);
        }

        // Start autonomous
        while (!isStopRequested()) {
            shooter.startShooterMotor(1000);
            shooter.startIntakeMotor(1000);
//            if (gamepad2.right_bumper) {
//                shooter.startShooterMotor(-45);
//                shooter.startIntakeMotor(45);
//            }
//            else if (gamepad2.right_trigger > 0.25) {
//                shooter.startIntakeMotor(45);
//            }
//            else if (gamepad2.left_trigger > 0.25) {
//                shooter.startShooterMotor(-45);
//            }
//            else {
//                shooter.startShooterMotor(0);
//                shooter.startIntakeMotor(0);
//            }
            dashboardTelemetry.addData("Shooter Motor", shooter.getShooterMotorVelocity());
            dashboardTelemetry.addData("Intake Motor", shooter.getIntakeMotorVelocity());
            dashboardTelemetry.update();
        }
    }
}
