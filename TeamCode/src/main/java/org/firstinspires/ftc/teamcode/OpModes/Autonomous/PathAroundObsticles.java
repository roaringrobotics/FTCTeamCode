package org.firstinspires.ftc.teamcode.OpModes.Autonomous;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotHardware.DriveTrain;
import org.firstinspires.ftc.teamcode.RobotHardware.PinpointImpl;

@Autonomous(group = "Test")
public class PathAroundObsticles extends LinearOpMode {
    private DriveTrain driveTrain;
    private PinpointImpl imu;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();


    private final ElapsedTime stateTimer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        driveTrain = new DriveTrain(hardwareMap);
        imu = new PinpointImpl(hardwareMap);

        // For pid testing
        double kp = 1.0;
        double ki = 0;
        double kd = 0;

        // For changing the field to match alliance
        boolean isRed = false;
        String team; // Displays our team

        // Other variables
        double power = 0.5;

        imu.reset();

        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Status", "Initialized");
            telemetry.addData("Hint", "Waiting for start - update sensors/vision here");
            telemetry.update();
            if (gamepad2.optionsWasPressed()) {
                isRed = !isRed;
            }
            if (isRed) {
                team = "Red Side";
            } else {
                team = "Blue Side";
            }

            telemetry.addData("Side", team);
            idle();
        }

        stateTimer.reset();

        driveTrain.driveToFieldCentric(-18, 112, 0, power, imu);
        driveTrain.driveToFieldCentric(30, 112, 180, power, imu);
        driveTrain.driveToFieldCentric(30, 44, 180, power, imu);
        driveTrain.driveToFieldCentric(0,10 , 0, power, imu);
        driveTrain.driveToFieldCentric(0,0 , 0, power, imu);


    }
}
