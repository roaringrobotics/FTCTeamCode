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

        // These values work for shooter motor
        double kps = 350.0;
        double kis = 2.0;
        double kds = 2.0;

        double kpi = 1.0;
        double kii = 2.0;
        double kdi = 2.0;

        // this velocity value works for the nectar
        double velocity = 1300;
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
            if (gamepad2.dpadDownWasPressed()) {
                velocity -= change;
            }
            if (gamepad2.dpadUpWasReleased()) {
                velocity += change;
            }

            if (gamepad2.a && gamepad2.dpadUpWasReleased()) {
                kds += change;
            } else if (gamepad2.a && gamepad2.dpadDownWasPressed()) {
                kds -= change;
            } else if (gamepad2.b && gamepad2.dpadUpWasReleased()) {
                kis += change;
            } else if (gamepad2.b && gamepad2.dpadDownWasPressed()) {
                kis -= change;
            } else if (gamepad2.y && gamepad2.dpadUpWasReleased()) {
                kps += change;
            } else if (gamepad2.y && gamepad2.dpadDownWasPressed()) {
                kps -= change;
            }

            telemetry.addData("PID Values", "kp: %.5f ki: %.5f kd: %.5f", kps, kis, kds);
            telemetry.addData("Velocity", velocity);
            telemetry.addData("Velocity Change", change);
            telemetry.update();
            shooter.setPidShooterValues(kps, kis, kds);
            shooter.setPidIntakeValues(kpi, kii, kdi);
        }

        // Start autonomous
        while (!isStopRequested()) {
            shooter.startShooterMotor(velocity);
            shooter.startIntakeMotor(velocity);

            dashboardTelemetry.addData("Shooter Motor", shooter.getShooterMotorVelocity());
            dashboardTelemetry.addData("Intake Motor", shooter.getIntakeMotorVelocity());
            dashboardTelemetry.update();
        }
    }
}
