package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Tests;

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
@Autonomous(name = "NoobieAuto", group = "Test")
public class NoobieAuto extends LinearOpMode {
    private DriveTrain driveTrain;
    private final ElapsedTime stateTimer = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        driveTrain = new DriveTrain(hardwareMap);
        boolean isRed = false;
        String team;
        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Status", "Initialized");

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

        stateTimer.reset();
        telemetry.addData("Status", "Started");
        telemetry.update();
        // Start autonomous
        double power = 0.5;
        driveTrain.driveToFieldCentric(63.5, 24, 90, power);
        driveTrain.driveToFieldCentric(63.5, 48, 90, power);
        //shoot here
        sleep(5000);
        driveTrain.driveToFieldCentric(63.5, 24, 90, power);
        driveTrain.driveToFieldCentric(3, 3, 0, power);
        driveTrain.driveToFieldCentric(-8.5, 3, 0, power);
    }
}
