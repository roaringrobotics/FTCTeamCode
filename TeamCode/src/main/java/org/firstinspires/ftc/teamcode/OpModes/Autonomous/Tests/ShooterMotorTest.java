package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Tests;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Implementations.AndroidLog;
import org.firstinspires.ftc.teamcode.Implementations.SystemTimeSource;
import org.firstinspires.ftc.teamcode.Pathing.PID;
import org.firstinspires.ftc.teamcode.Pathing.PowerRampController;
import org.firstinspires.ftc.teamcode.RobotHardware.Intake;

import com.acmerobotics.dashboard.FtcDashboard;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.RobotHardware.Shooter;

// @Autonomous(name = "Shooter Motor PID", group = "Test")
public class ShooterMotorTest extends LinearOpMode {
    private Shooter shooter;
    private Intake intake;
    private final AndroidLog log = new AndroidLog();

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    @Override
    public void runOpMode() {
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

        double kp = 500.0;
        double ki = 0.0;
        double kd = 0.0;
        double change = 10;

        double targetDistance = 0;


        // show init telemetry until start pressed
        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Status", "Initialized");
            telemetry.addData("Hint", "Waiting for start - update sensors/vision here");


            if (gamepad2.dpadUpWasPressed() && gamepad2.a) kp += change;
            else if (gamepad2.dpadDownWasPressed() && gamepad2.a) kp -= change;

            if (gamepad2.dpadUpWasPressed() && gamepad2.b) kd += change;
            else if (gamepad2.dpadDownWasPressed() && gamepad2.b) kd -= change;

            if (gamepad2.rightBumperWasPressed()) change *= 10;
            else if (gamepad2.leftBumperWasPressed()) change *= 0.1;

            telemetry.addData("PID Values", "kp: %.5f ki: %.5f kd: %.5f", kp, ki, kd);
            telemetry.addData("PID Change", change);
            telemetry.update();
            shooter.setPIDValues(kp, ki, kd);
        }



        // these are for strafing
        double kps = 0.13;
        double kds = 0.29;

        // Start autonomous
        while (!isStopRequested()) {
            shooter.startShooterMotor(1000);
            double velocity = shooter.getShooterVelocity();
            dashboardTelemetry.addData("Velocity", velocity);
            dashboardTelemetry.addData("kp", kp);
            dashboardTelemetry.addData("kd", kd);
            dashboardTelemetry.update();

        }
    }
}
