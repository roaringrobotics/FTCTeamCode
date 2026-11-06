package org.firstinspires.ftc.teamcode.Interfaces;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

// Interface for accessing hardware that provides
// header and position information.
// Implemented by ImuImpl and PinpointImpl.
public interface ImuPositionI {
    // Reset the IMU heading and position if possible.
    void reset() throws InterruptedException;

    void resetHeading();

    // Get heading in degrees.
    double getHeading(AngleUnit units);
    // Get Y position in inches.  +Y is forward -Y is backward relative to robot.
    double getPosY();
    // Get X position in inches.  +X is left -X is right relative to robot.
    double getPosX();
    // Called to read information from hardware if needed.
    // Users of ImuPositionWrapper should always call this before reading new values.
    void update();

    Pose2D getPose() throws Exception;
}

