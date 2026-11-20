package org.firstinspires.ftc.teamcode.Math;

public class Util {
    public static int clamp(int value, int min, int max) {
        return Math.max(min, Math.min(value, max));
    }

    public static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(value, max));
    }

    // This function normalizes the angle so it returns a value between -pi and pi instead of 0 to 2*pi.
    public static double angleWrapRadians(double radians) {

        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }

        return radians;
    }
    // This function normalizes the angle so it returns a value between -180° and 180° instead of 0° to 360°.
    public static double angleWrapDegrees(double degrees) {

        while (degrees > 180) {
            degrees -= 360;
        }
        while (degrees < -180) {
            degrees += 360;
        }

        return degrees;
    }
}
