package org.firstinspires.ftc.teamcode.RobotHardware;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Interfaces.HardwareI;
import org.firstinspires.ftc.teamcode.Interfaces.ImuPositionI;
import org.firstinspires.ftc.teamcode.Interfaces.TimeSourceI;

// Initializes and tracks the Robot Hardware.
// This class should be removed soon
public class Hardware {



    public Hardware(HardwareMap hardwareMap) {
        InitImu(hardwareMap);
    } // only Imu stuff

    private void InitImu(HardwareMap hardwareMap) {
        // Initialize IMU and Encoders.  imuPos can be internal IMU/Encoders or Pinpoint computer.
        // imuPos = new ImuImp(hardwareMap);
        imuPos = new PinpointImpl(hardwareMap); // use pinpoint
    }


    // ImuWrapper is implemented for IMU or Pinpoint.
    public ImuPositionI imuPos;



    public void updateImuPos() {
        imuPos.update();
    }


    public void resetImu() throws InterruptedException {
        imuPos.reset();
    }


    public void resetImuHeading() {
        imuPos.resetHeading();
    }


    public Pose2D getImuPose() {
        try {
            return imuPos.getPose();
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
    }

    public double getImuHeading(AngleUnit unit) {
        return imuPos.getHeading(unit);
    }

    public void updateState(TimeSourceI timeSource) {
        // No needed when running robot.
        // Maybe could be useful in the future?
    }
}
