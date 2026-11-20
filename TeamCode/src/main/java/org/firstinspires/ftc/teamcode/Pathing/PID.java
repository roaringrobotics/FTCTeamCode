package org.firstinspires.ftc.teamcode.Pathing;

import org.firstinspires.ftc.teamcode.Math.Util;

public class PID {
    private double kP, kI, kD;
    private double previousError = 0;
    private double integral = 0;

    public PID(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public double calculate(double target, double current) {
        double error = target - current;

//        These lines keep causing undefined values, will be fixed later
//        integral += error;
//        integral = Util.clamp(integral, -1.0, 1.0);

        double derivative = error - previousError;
        previousError = error;

        return kP * error + kI * integral + kD * derivative;
    }

    // This function is for if we have already calculated the error
    public double calculate(double error) {
//        These lines keep causing undefined values, will be fixed later
//        integral += error;
//        integral = Util.clamp(integral, -1.0, 1.0);

        double derivative = error - previousError;
        previousError = error;

        return kP * error + kI * integral + kD * derivative;
    }

    public void reset() {
        previousError = 0;
        integral = 0;
    }
}

