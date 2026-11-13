package org.firstinspires.ftc.teamcode.PreviousYearsCode.DecodeOpModes.DecodeOpModes.TeleOp;


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
@TeleOp(group = "Decode")
public class DecodeSingleTeleop extends LinearOpMode {
    private DriveTrain driveTrain;
    private Shooter shooter;
    private PinpointImpl imu;
    private Intake intake;
    private SystemTimeSource timesource = new SystemTimeSource();
    private final AndroidLog log = new AndroidLog();

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    // 3. Add a timer to debounce the button


    public void runOpMode() throws InterruptedException {
        driveTrain = new DriveTrain(hardwareMap);
        imu = new PinpointImpl(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap, intake, log);
        //  intake = new Intake(hardwareMap);
        waitForStart();
        float deadZone = 0.5F;
        while (opModeIsActive()) {

            double fieldY = gamepad1.left_stick_y;
            double fieldX = gamepad1.left_stick_x; // the negative is a hotfix
            double rotate = gamepad1.right_stick_x;
            dashboardTelemetry.addData("fieldX", fieldX);
            dashboardTelemetry.addData("fieldY", fieldY);
            dashboardTelemetry.addData("rotate", rotate);
            dashboardTelemetry.update();
            double powerScale = driveTrain.powerScaler(gamepad1.right_bumper, gamepad1.left_bumper);
            imu.update();
//            log.d("Heading: ", String.valueOf(hw.imuPos.getHeading(AngleUnit.DEGREES)));


            driveTrain.driveFieldCentric(fieldX, fieldY, rotate, powerScale, imu);

            timesource.update();

            if (gamepad1.dpad_down)
                shooter.singleShoot(gamepad1.a, gamepad1.b, gamepad1.y);
            else
                shooter.continousShoot(gamepad1.a, gamepad1.b, gamepad1.y);

            if (!gamepad1.a && !gamepad1.b && !gamepad1.y) {

                if (gamepad1.right_trigger > deadZone)
                    shooter.startIntake();
                else if (gamepad1.left_trigger > deadZone)
                    shooter.reverseIntake();
                else
                    shooter.stopIntake();

                if (gamepad1.dpad_left)
                    shooter.reverseShootServos();
                else if (gamepad1.dpad_right)
                    shooter.startShootServos();
                else
                    shooter.stopShootServos();


            }
            if (gamepad1.options) {
                imu.reset();
            }

        }

    }
}

