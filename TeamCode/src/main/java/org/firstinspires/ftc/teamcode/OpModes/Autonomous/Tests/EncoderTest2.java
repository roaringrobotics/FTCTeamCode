


package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.RobotHardware.DriveTrain;
import org.firstinspires.ftc.teamcode.RobotHardware.PinpointImpl;

// This program gives good imu readings, driveToRelative does not, why?
@Autonomous(name = "IMU Test Auto", group = "Test")
public class EncoderTest2 extends LinearOpMode {
    private DriveTrain driveTrain;
    private PinpointImpl imu;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    public void runOpMode() throws InterruptedException {
        driveTrain = new DriveTrain(hardwareMap);
        imu = new PinpointImpl(hardwareMap);
        imu.reset();
        double powerScale = 0.25;

        waitForStart();
        while (opModeIsActive() && powerScale > 0.0) {
                imu.update();
                Pose2D pose = imu.getPose();

                driveTrain.setFrontRightPower(powerScale);
                driveTrain.setFrontLeftPower(-powerScale);
                driveTrain.setBackRightPower(powerScale);
                driveTrain.setBackLeftPower(-powerScale);

                dashboardTelemetry.addData("Pose",
                        String.format("HeadingDegrees: %.3f | PoseX: %.3f | PoseY: %.3f | Power: %.4f",
                                pose.getHeading(AngleUnit.DEGREES),
                                pose.getX(DistanceUnit.INCH),
                                pose.getY(DistanceUnit.INCH), powerScale));
                dashboardTelemetry.update();
                sleep(1);

        }
    }
}
