// java
package org.firstinspires.ftc.teamcode.PreviousYearsCode.DecodeRobotHardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    private final DcMotor motor;

    // Default constructor uses device name "intakeMotor"
    public Intake(HardwareMap hardwareMap) {
        this(hardwareMap, "intake");
    }

    // Constructor allowing a custom device name
    public Intake(HardwareMap hardwareMap, String deviceName) {
        motor = hardwareMap.get(DcMotor.class, deviceName);

        // Default configuration: no direction reversal here (change if needed)
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setPower(double power) {
        motor.setPower(power);
    }

    public double getPower() {
        return motor.getPower();
    }

    public void stop() {
        setPower(0.0);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        motor.setZeroPowerBehavior(behavior);
    }

    public void setDirection(DcMotorSimple.Direction direction) {
        motor.setDirection(direction);
    }

    public void setRunMode(DcMotor.RunMode mode) {
        motor.setMode(mode);
    }

    public void startIntake() {
        setPower(-1.0);
    }
    public void stopIntake() {
        setPower(0.0);
    }
    public void reverseIntake(){setPower(1.0);}
}
