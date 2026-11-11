package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.RobotHardware.DriveTrain;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Implementations.AndroidLog;
import org.firstinspires.ftc.teamcode.Implementations.SystemTimeSource;
import org.firstinspires.ftc.teamcode.RobotHardware.DriveTrain;
import org.firstinspires.ftc.teamcode.RobotHardware.Hardware;

import org.firstinspires.ftc.teamcode.RobotHardware.Intake;
import org.firstinspires.ftc.teamcode.RobotHardware.PinpointImpl;
import org.firstinspires.ftc.teamcode.RobotHardware.Shooter;

    @TeleOp
    public class TeleOpBiobuzz extends LinearOpMode {
        private DriveTrain driveTrain;
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();
        private PinpointImpl imu;
        public void runOpMode() throws InterruptedException {
            driveTrain = new DriveTrain(hardwareMap);
            imu = new PinpointImpl(hardwareMap);
            imu.reset();
            waitForStart();
            driveTrain.driveToFieldCentric(0, 12, 90, 0.5, imu);
            imu.reset();
            sleep(300);
            while (opModeIsActive()) {
                double fieldX = gamepad1.left_stick_x;
                double fieldY = gamepad1.left_stick_y;
                double rotate = gamepad1.right_stick_x;
                double powerScale = driveTrain.powerScaler(gamepad1.right_bumper, gamepad1.left_bumper);

                driveTrain.driveFieldCentric(fieldX, fieldY, rotate, powerScale, imu);

                if (gamepad1.options) {
                    imu.reset();
                }
            }
        }
}
