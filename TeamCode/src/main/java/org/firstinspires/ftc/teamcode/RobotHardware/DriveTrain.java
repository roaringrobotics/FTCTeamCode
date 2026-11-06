package org.firstinspires.ftc.teamcode.RobotHardware;

import static java.lang.Thread.sleep;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Implementations.AndroidLog;
import org.firstinspires.ftc.teamcode.Interfaces.LogI;
import org.firstinspires.ftc.teamcode.Interfaces.ImuPositionI;
import org.firstinspires.ftc.teamcode.Math.Util;
import org.firstinspires.ftc.teamcode.Math.Vector2;
import org.firstinspires.ftc.teamcode.Pathing.PID;

public class DriveTrain {
    public DcMotor frontRight;
    public DcMotor frontLeft;
    public DcMotor backLeft;
    public DcMotor backRight;
    public FieldCentricPowerLevels fcPowerLevels = new FieldCentricPowerLevels();
    private final AndroidLog log = new AndroidLog();

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    public DriveTrain(HardwareMap hardwareMap)
    {

        // Drive Motors
        // Names need to match configuration on driver hub
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backRight = hardwareMap.dcMotor.get("backRight");


        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void setFrontLeftPower(double power) {
        power = Math.min(power, 1.0);
        power = Math.max(power, -1.0);
        frontLeft.setPower(power);
    }

    public void setFrontRightPower(double power) {
        power = Math.min(power, 1.0);
        power = Math.max(power, -1.0);
        frontRight.setPower(power);

    }

    public void setBackLeftPower(double power) {
        power = Math.min(power, 1.0);
        power = Math.max(power, -1.0);
        backLeft.setPower(power);

    }

    public void setBackRightPower(double power) {
        power = Math.min(power, 1.0);
        power = Math.max(power, -1.0);
        backRight.setPower(power);
    }

    public double getFrontLeftPower() {
        return frontLeft.getPower();
    }

    public double getFrontRightPower() {
        return frontRight.getPower();
    }

    public double getBackLeftPower() {
        return backLeft.getPower();
    }

    public double getBackRightPower() {
        return backRight.getPower();
    }

    public void stopMotors() {
        setFrontLeftPower(0);
        setFrontRightPower(0);
        setBackLeftPower(0);
        setBackRightPower(0);
    }

     public static class FieldCentricPowerLevels {
        public double frontLeftPower;
        public double backLeftPower;
        public double frontRightPower;
        public double backRightPower;

        public FieldCentricPowerLevels() {

        }
    }
    public void driveFieldCentric(double fieldX, double fieldY, double rightStickX,
                                   double powerScale, Hardware hw) {
        // Takes joystick input and sets motor power levels to drive
        // robot field centric.
        // fieldX is the field centric away/towards driver stick input (forward/back)
        // fieldY is the field centric horizontal movement.
        hw.imuPos.update();
        double botHeading = hw.imuPos.getHeading(AngleUnit.RADIANS);

        getFieldCentricPowerLevels(
                fieldX, fieldY,
                rightStickX, botHeading);

        frontLeft.setPower(fcPowerLevels.frontLeftPower * powerScale);
        backLeft.setPower(fcPowerLevels.backLeftPower * powerScale);
        frontRight.setPower(fcPowerLevels.frontRightPower * powerScale);
        backRight.setPower(fcPowerLevels.backRightPower * powerScale);
    }
    @SuppressLint("DefaultLocale")
    private void getFieldCentricPowerLevels(
            double fieldX, // + away from driver, - towards driver
            double fieldY, // + left, - right
            double rotate, double botHeading) {

        // Delegate computation to testable static helper
        FieldCentricPowerLevels result = computeFieldCentricPowerLevels(fieldX, fieldY, rotate, botHeading);
        // copy into instance field
        fcPowerLevels.frontLeftPower = result.frontLeftPower;
        fcPowerLevels.backLeftPower = result.backLeftPower;
        fcPowerLevels.frontRightPower = result.frontRightPower;
        fcPowerLevels.backRightPower = result.backRightPower;

        dashboardTelemetry.addData("FC",
                String.format("Y: %.2f X: %.2f R: %.2f | FL:%.2f FR:%.2f BL:%.2f BR:%.2f, H: %.3f",
                fieldY, fieldX, rotate,
                fcPowerLevels.frontLeftPower, fcPowerLevels.frontRightPower,
                fcPowerLevels.backLeftPower, fcPowerLevels.backRightPower,
                Math.toDegrees(botHeading)));
        dashboardTelemetry.update();
    }

    // New testable static computation method – performs same math but without telemetry or instance state
    public static FieldCentricPowerLevels computeFieldCentricPowerLevels(
            double fieldX,
            double fieldY,
            double rotate,
            double botHeading) {
        FieldCentricPowerLevels out = new FieldCentricPowerLevels();

        // Invert Y axis if needed.  Using gobilda pinpoint convention.
        double fy = -fieldY;
        double fx = -fieldX;

        // Y is forward (vertical) direction, X is the strafe (horizontal direction)
        double robotY = fy * Math.cos(-botHeading) + fieldX * Math.sin(-botHeading);
        double robotX = -fy * Math.sin(-botHeading) + fieldX * Math.cos(-botHeading);

        // Normalize output power [-1.0-1.0]
        double vectorSum = Math.abs(robotX) + Math.abs(robotY) + Math.abs(rotate);
        double denom = Math.max(vectorSum, 1.0);

//        out.frontLeftPower = (robotY + robotX + rotate) / denom;
//        out.backLeftPower = (-robotY + robotX + rotate) / denom;
//        out.frontRightPower = (-robotY + robotX - rotate) / denom;
//        out.backRightPower = (robotY + robotX - rotate) / denom;

        out.frontLeftPower = (robotY + robotX + rotate) / denom;
        out.backLeftPower = (robotY - robotX + rotate) / denom;
        out.frontRightPower = (robotY - robotX - rotate) / denom;
        out.backRightPower = (robotY + robotX - rotate) / denom;

        return out;
    }

    // Class to hold field centric power level output from getFieldCentricPowerLevels
    @SuppressLint("DefaultLocale")
    public void driveStraight(double distance, double power, ImuPositionI imu, LogI log,
                              double kP, double kI, double kD) throws Exception {
        imu.update();
        Pose2D startPose = imu.getPose();
        double startHeadingDeg = imu.getHeading(AngleUnit.DEGREES);

        // Target pose (used only for computing delta to final point)
        Pose2D targetPose = new Pose2D(
                DistanceUnit.INCH,
                startPose.getX(DistanceUnit.INCH) + distance * Math.cos(Math.toRadians(startHeadingDeg)),
                startPose.getY(DistanceUnit.INCH) + distance * Math.sin(Math.toRadians(startHeadingDeg)),
                AngleUnit.DEGREES, startHeadingDeg);

        PID pid = new PID(kP, kI, kD);
        final double distanceTolerance = 0.25; // inches
        final double minDrivePower = 0.15;     // minimum drive power to overcome static friction
        final int settleCountsRequired = 3;

        // Small PD for heading correction
        final double headingKp = 0.02;
        final double headingKd = 0.002;
        double lastHeadingError = 0.0;

        int settleCount = 0;
        double distanceTraveled = 0.0;
        double delta = Vector2.distanceBetweenPoses(targetPose, startPose); // initial delta ~= abs(distance)

        long lastTime = System.nanoTime();

        while (true) {
            imu.update();
            long now = System.nanoTime();
            double dt = Math.max(1e-6, (now - lastTime) / 1e9);
            lastTime = now;

            Pose2D curPose = imu.getPose();
            double curHeading = imu.getHeading(AngleUnit.DEGREES);

            distanceTraveled = Vector2.distanceBetweenPoses(startPose, curPose);
            // remaining distance to target
            delta = Vector2.distanceBetweenPoses(targetPose, curPose);

            // Determine motion direction sign (forward/backward)
            // If requested distance was negative, we should drive backwards.
            double sign = distance >= 0 ? 1.0 : -1.0;
            double remaining = Math.abs(distance) - distanceTraveled;

            // Check settle condition
            if (Math.abs(remaining) <= distanceTolerance) {
                settleCount++;
                if (settleCount >= settleCountsRequired) break;
            } else {
                settleCount = 0;
            }

            // PID: many project PID.calculate(setpoint, current)
            double basePower = pid.calculate(Math.abs(distance), distanceTraveled);
            // Ensure signed power for forward/backward
            basePower = basePower * sign;

            // Apply minimum drive power only when still away from target
            if (Math.abs(remaining) > distanceTolerance && Math.abs(basePower) < minDrivePower) {
                basePower = minDrivePower * Math.signum(basePower != 0.0 ? basePower : sign);
            }

            // Heading correction to keep straight using initial heading
            double headingError = startHeadingDeg - curHeading;
            // normalize to [-180,180]
            while (headingError > 180) headingError -= 360;
            while (headingError <= -180) headingError += 360;

            double headingDerivative = (headingError - lastHeadingError) / dt;
            lastHeadingError = headingError;

            double headingCorrection = headingKp * headingError + headingKd * headingDerivative;
            // Apply heading correction to left/right (subtract from left, add to right for correction)
            double leftPower = basePower - headingCorrection;
            double rightPower = basePower + headingCorrection;

            // Clamp powers
            leftPower = Math.max(-power, Math.min(power, leftPower));
            rightPower = Math.max(-power, Math.min(power, rightPower));

            // Set motor powers (use set* methods so motor direction config is respected)
            setFrontLeftPower(leftPower);
            setBackLeftPower(leftPower);
            setFrontRightPower(rightPower);
            setBackRightPower(rightPower);

            if (log != null) {
                log.d("DriveStraight", String.format("rem: %.3f trav: %.3f base: %.3f L: %.3f R: %.3f hErr: %.2f",
                        remaining, distanceTraveled, basePower, leftPower, rightPower, headingError));
            }

            // Loop cadence
            sleep(20);
        }

        stopMotors();
        if (log != null) {
            log.d("DriveStraight", "Complete. Traveled: " + distanceTraveled);
        }
    }
    @SuppressLint("DefaultLocale")
    public void driveToRelative(double x,
                                double y,
                                double endDegrees,
                                double power,
                                boolean isRed,
                                ImuPositionI imu, double kp, double ki, double kd) throws Exception {
        imu.update();
        Pose2D startPose = imu.getPose();

        if (isRed) {
            x = -x;
            endDegrees = -endDegrees;
        }
        // Calculate target pose in field coordinates
        Pose2D targetPose = new Pose2D(
                DistanceUnit.INCH,
                startPose.getX(DistanceUnit.INCH) + x,
                startPose.getY(DistanceUnit.INCH) + y,
                AngleUnit.DEGREES,
                endDegrees);

//        double linearPercentage;
//        double roationPercentage;
        double targetDistance = Math.sqrt(x * x + y * y);

        // PID controllers
        PID drivePID = new PID(kp, ki, 0);     // For position control
        PID rotatePID = new PID(0.9, ki, kd); // For rotation control

        final double distanceTolerance = 0.5;  // inches
        final double angleTolerance = 2.0;     // degrees
        final double minDrivePower = 0.12;
        final double minRotatePower = 0.15;
        final double maxDrivePower = 1.0;
        final double maxRotatePower = 1.0;
        final int settleCountsRequired = 5;

        int settleCount = 0;
        long lastTime = System.nanoTime();

        while (true) {
            imu.update();
            long now = System.nanoTime();
            double dt = Math.max(1e-6, (now - lastTime) / 1e9);
            lastTime = now;

            Pose2D curPose = imu.getPose();
            double curHeading = curPose.getHeading(AngleUnit.DEGREES);

            // Calculate remaining distance to target
            double deltaX = targetPose.getX(DistanceUnit.INCH) - curPose.getX(DistanceUnit.INCH);
            double deltaY = targetPose.getY(DistanceUnit.INCH) - curPose.getY(DistanceUnit.INCH);
            double remainingDistance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);

            // Calculate heading error
            double headingError = endDegrees - curHeading;
            while (headingError > 180) headingError -= 360;
            while (headingError <= -180) headingError += 360;

            // Check if we're on target
            boolean positionOnTarget = remainingDistance <= distanceTolerance;
            boolean headingOnTarget = Math.abs(headingError) <= angleTolerance;

            if (positionOnTarget && headingOnTarget) {
                settleCount++;
                if (settleCount >= settleCountsRequired) break;
            } else {
                settleCount = 0;
            }

            // Calculate drive vector in field frame
            double driveX = 0.0;
            double driveY = 0.0;

            if (!positionOnTarget) {
                // PID control based on remaining distance
                double driveMagnitude = drivePID.calculate(targetDistance, targetDistance - remainingDistance);
                driveMagnitude = Util.clamp(driveMagnitude, -1, 1);

                // Apply minimum power
                if (Math.abs(driveMagnitude) < minDrivePower) {
                    driveMagnitude = minDrivePower * Math.signum(driveMagnitude);
                }

                // Direction vector (normalized)
                double dirX = deltaX / remainingDistance;
                double dirY = deltaY / remainingDistance;

                driveX = dirY * driveMagnitude;
                driveY = dirX * driveMagnitude;
            }

            // Calculate rotation power
            double rotatePower = 0.0;
            if (!headingOnTarget) {
                rotatePower = rotatePID.calculate(endDegrees, curHeading);
                rotatePower = Util.clamp(rotatePower, -1, 1);
                // Apply minimum rotation power
                if (Math.abs(rotatePower) < minRotatePower && Math.abs(headingError) > angleTolerance) {
                    rotatePower = minRotatePower * Math.signum(rotatePower);
                }
                if (Math.abs(rotatePower) > maxRotatePower) {
                    rotatePower = maxRotatePower * Math.signum(rotatePower);
                }
            }

            double botHeadingRad = Math.toRadians(curHeading);
            double robotX = driveY * Math.cos(-botHeadingRad) - driveX * Math.sin(-botHeadingRad);
            double robotY = driveY * Math.sin(-botHeadingRad) + driveX * Math.cos(-botHeadingRad);


            // Calculate mecanum wheel powers
            double vectorSum = Math.abs(robotY) + Math.abs(robotX) + Math.abs(rotatePower);
            double denom = Math.max(vectorSum, 1.0);

            // Testing values for rotation
//            robotY = 0.0;
//            robotX = 0.0;

            double flPower = (robotY + robotX - rotatePower) / denom;
            double blPower = (robotY - robotX - rotatePower) / denom;
            double frPower = (robotY - robotX + rotatePower) / denom;
            double brPower = (robotY + robotX + rotatePower) / denom;

            // Apply power limit and signs (matching your motor configuration)

            setFrontLeftPower(flPower * power);
            setBackLeftPower(blPower * power);
            setFrontRightPower(frPower * power);
            setBackRightPower(brPower * power);

            dashboardTelemetry.addData("DriveLinear",
                    String.format("endDegrees: %.3f | curDegrees: %.3f | error %.3f | hot %b | pot %b | sc %d",
                    endDegrees, curHeading, headingError, headingOnTarget, positionOnTarget, settleCount));

            dashboardTelemetry.addData("Power Levels",
                    String.format("robotY: %.2f | robotX: %.2f | rotatePower %.2f | demon: %.2f",
                    robotY, robotX, rotatePower, denom));

            dashboardTelemetry.addData("Location",
                    String.format("x: %.2f | y: %.2f",
                            curPose.getX(DistanceUnit.INCH), curPose.getY(DistanceUnit.INCH)));


            dashboardTelemetry.update();

            sleep(1);
        }

        stopMotors();
        if (log != null) {
            log.d("DriveLinear", "Complete - reached target position and heading");
        }
    }

    public static double clamp(double value, double min, double max) {
        if (min > max) throw new IllegalArgumentException("min must be <= max");
        return Math.max(min, Math.min(max, value));
    }
    public void driveFieldCentric(double x, double y, double endDegrees, double power, PinpointImpl imu) {
        Pose2D pos = imu.getPose();
        x = x - pos.getX(DistanceUnit.INCH);
        y = y - pos.getY(DistanceUnit.INCH);


        try {
            driveToRelative(x, y, endDegrees, power, false, imu, 1, 0, 0);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }

    }

    /**
     * Rotates the robot in place by a relative angle (positive = clockwise, negative = counterclockwise).
     * Uses PID control for smooth and accurate turns.
     *
     * @param relativeAngleDeg The angle to rotate relative to current heading (e.g., +90 for 90° right)
     * @param imu              IMU interface for getting current heading
     * @param log              Logger for debugging (optional, can be null if not needed)
     */
    @SuppressLint("DefaultLocale")
    public void rotateRelative(double relativeAngleDeg, ImuPositionI imu, LogI log) throws Exception {
        imu.update();
        double currentHeading = imu.getHeading(AngleUnit.DEGREES);

        // Calculate absolute target heading
        double targetHeading = currentHeading + relativeAngleDeg;

        // Normalize target to [-180, 180] or [0, 360] — here we keep it continuous but normalize error later
        // No need to normalize target itself, only the error

        PID pid = new PID(0.01, 0, 0); // Tune these: Kp, Ki, Kd
        double minPower = 0.18;       // Minimum power to overcome friction — adjust based on your robot
        double toleranceDeg = 2.0;    // Consider "on target" if within this
        int settleCountsRequired = 3; // Must stay within tolerance for ~15 loops (~300ms at 20ms loop)
        int settleCount = 0;

        while (true) {
            imu.update();
            currentHeading = imu.getHeading(AngleUnit.DEGREES);

            // Calculate error with shortest path (-180 to +180)
            double error = targetHeading - currentHeading;
            while (error > 180) error -= 360;
            while (error <= -180) error += 360;

            // Exit condition: small error and settled
            if (Math.abs(error) <= toleranceDeg) {
                settleCount++;
                if (settleCount >= settleCountsRequired) {
                    break;
                }
            } else {
                settleCount = 0;
            }

            // PID calculation — assuming your PID.calculate(setpoint, current) returns correction
            // Many FTC PID classes use calculate(target, current), so:
            double power = pid.calculate(targetHeading, currentHeading);

            // Alternative: if your PID expects error directly or setpoint=0:
            // double power = pid.calculate(0, error);

            // Apply minimum power when needed (preserve direction)
            if (Math.abs(power) < minPower && Math.abs(error) > toleranceDeg) {
                power = minPower * Math.signum(power);
            }

            // Clamp power to motor limits
            power = Math.max(-1.0, Math.min(1.0, power));

            // Apply tank turn: positive power = clockwise rotation (common convention)
            setFrontLeftPower(-power);
            setBackLeftPower(-power);
            setFrontRightPower(power);
            setBackRightPower(power);

            // Optional logging
            if (log != null) {
                log.d("RotateRel", "Target: " + String.format("%.1f", targetHeading) +
                        " | Current: " + String.format("%.1f", currentHeading) +
                        " | Error: " + String.format("%.1f", error) +
                        " | Power: " + String.format("%.3f", power));
            }

            sleep(20); // 50 Hz loop — stable and efficient
        }

        stopMotors();

        if (log != null) {
            log.d("RotateRel", "Rotation complete. Final heading: " + currentHeading);
        }
    }

    @SuppressLint("DefaultLocale")
    public void driveStrafe(double distance, double power, ImuPositionI imu, LogI log,
                            double kP, double kI, double kD) throws Exception {
        imu.update();
        Pose2D startPose = imu.getPose();
        double startHeadingDeg = imu.getHeading(AngleUnit.DEGREES);

        // Target pose (used only for computing delta to final point)
        Pose2D targetPose = new Pose2D(
                DistanceUnit.INCH,
                startPose.getX(DistanceUnit.INCH) + distance * Math.cos(Math.toRadians(startHeadingDeg)),
                startPose.getY(DistanceUnit.INCH) + distance * Math.sin(Math.toRadians(startHeadingDeg)),
                AngleUnit.DEGREES, startHeadingDeg);

        PID pid = new PID(kP, kI, kD);
        final double distanceTolerance = 0.25; // inches
        final double minDrivePower = 0.15;     // minimum drive power to overcome static friction
        final int settleCountsRequired = 3;

        // Small PD for heading correction
        final double headingKp = 0.02;
        final double headingKd = 0.002;
        double lastHeadingError = 0.0;

        int settleCount = 0;
        double distanceTraveled = 0.0;
        double delta = Vector2.distanceBetweenPoses(targetPose, startPose); // initial delta ~= abs(distance)

        long lastTime = System.nanoTime();

        while (true) {
            imu.update();
            long now = System.nanoTime();
            double dt = Math.max(1e-6, (now - lastTime) / 1e9);
            lastTime = now;

            Pose2D curPose = imu.getPose();
            double curHeading = imu.getHeading(AngleUnit.DEGREES);

            distanceTraveled = Vector2.distanceBetweenPoses(startPose, curPose);
            // remaining distance to target
            delta = Vector2.distanceBetweenPoses(targetPose, curPose);

            // Determine motion direction sign (forward/backward)
            // If requested distance was negative, we should drive backwards.
            double sign = distance >= 0 ? 1.0 : -1.0;
            double remaining = Math.abs(distance) - distanceTraveled;

            // Check settle condition
            if (Math.abs(remaining) <= distanceTolerance) {
                settleCount++;
                if (settleCount >= settleCountsRequired) break;
            } else {
                settleCount = 0;
            }

            // PID: many project PID.calculate(setpoint, current)
            double basePower = pid.calculate(Math.abs(distance), distanceTraveled);
            // Ensure signed power for forward/backward
            basePower = basePower * sign;

            // Apply minimum drive power only when still away from target
            if (Math.abs(remaining) > distanceTolerance && Math.abs(basePower) < minDrivePower) {
                basePower = minDrivePower * Math.signum(basePower != 0.0 ? basePower : sign);
            }

            // Heading correction to keep straight using initial heading
            double headingError = startHeadingDeg - curHeading;
            // normalize to [-180,180]
            while (headingError > 180) headingError -= 360;
            while (headingError <= -180) headingError += 360;

            double headingDerivative = (headingError - lastHeadingError) / dt;
            lastHeadingError = headingError;

            double headingCorrection = headingKp * headingError + headingKd * headingDerivative;
            // Apply heading correction to left/right (subtract from left, add to right for correction)
            double leftPower = basePower - headingCorrection;
            double rightPower = basePower + headingCorrection;

            // Clamp powers
            leftPower = Math.max(-power, Math.min(power, leftPower));
            rightPower = Math.max(-power, Math.min(power, rightPower));

            // Set motor powers (use set* methods so motor direction config is respected)
            setFrontLeftPower(-leftPower);
            setBackLeftPower(leftPower);
            setFrontRightPower(rightPower);
            setBackRightPower(-rightPower);

            if (log != null) {
                log.d("DriveStraight", String.format("rem: %.3f trav: %.3f base: %.3f L: %.3f R: %.3f hErr: %.2f",
                        remaining, distanceTraveled, basePower, leftPower, rightPower, headingError));
            }

            // Loop cadence
            sleep(20);
        }

        stopMotors();
        if (log != null) {
            log.d("DriveStrafe", "Complete. Traveled: " + distanceTraveled);
        }
    }

    public double powerScaler(boolean halfSpeed, boolean quarterSpeed) {
        double powerScale;
        if (halfSpeed) {
            powerScale = 0.5;
        } else if (quarterSpeed) {
            powerScale = 0.25;
        } else {
            powerScale = 1.0;
        }
        return powerScale;
    }
    public void driveRobotCentric(double robotX, double robotY, double rightStickX, // rightStickX rotates the robot
                                  double powerScale){

        double vectorSum = Math.abs(robotX) + Math.abs(robotY) + Math.abs(rightStickX);
        double denom = Math.max(vectorSum, 1.0);

        double frontLeftPower = (robotY - robotX + rightStickX); // denom;
        double backLeftPower = (robotY + robotX + rightStickX); // denom;
        double frontRightPower = (robotY - robotX - rightStickX); // denom;
        double backRightPower = (robotY + robotX - rightStickX); // denom;

        dashboardTelemetry.addData("Front Left Power", frontLeftPower);
        dashboardTelemetry.addData("Back Left Power", backLeftPower);
        dashboardTelemetry.addData("Front Right Power", frontRightPower);
        dashboardTelemetry.addData("Back Right Power", backRightPower);
        dashboardTelemetry.update();

        setFrontLeftPower(frontLeftPower * powerScale);
        setBackLeftPower(backLeftPower * powerScale);
        setBackRightPower(frontRightPower * powerScale);
        setFrontRightPower(backRightPower * powerScale);
    }

}
