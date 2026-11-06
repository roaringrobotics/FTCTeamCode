package org.firstinspires.ftc.teamcode.RobotHardware;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Interfaces.ImuPositionI;

// ImuPostionWrapper implementation for GoBilda Pinpoint computer.

public class PinpointImpl implements ImuPositionI {
    private final GoBildaPinpointDriver pp;

    public PinpointImpl(HardwareMap hwmap) {
        pp = hwmap.get(GoBildaPinpointDriver.class, "pinpoint");
        pp.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pp.setEncoderResolution(Constants.encoderResolution);
        pp.setOffsets(-165.1, 71.5);
    }

    @Override
    public void reset() throws InterruptedException {
        pp.resetPosAndIMU();
        sleep(500);
    }

    @Override
    public void resetHeading()
    {
        pp.recalibrateIMU();
    }

    @Override
    public double getHeading(AngleUnit units) {
        if(units == AngleUnit.DEGREES)
            return Math.toDegrees(pp.getHeading());
        else
            return pp.getHeading();
    }

    // swap the X Y from the PP so X is left/right and Y is forward/backward
    @Override
    public double getPosY() {
        return pp.getPosY() * Constants.mmToInch;
    }

    @Override
    public double getPosX() {
        return pp.getPosX () * Constants.mmToInch;
    }

    @Override
    public void update() {
        pp.update();
    }

    @Override
    public Pose2D getPose() {
        return pp.getPosition();
    }

}
