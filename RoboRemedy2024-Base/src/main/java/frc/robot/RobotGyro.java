// RobotGyro.java - Adds an offset to the NavX gyro
package frc.robot;

import edu.wpi.first.wpilibj.*;
import com.kauailabs.navx.frc.*;

public class RobotGyro extends AHRS {

    private double gyroOffset = 0;

    RobotGyro() {
        super(SPI.Port.kMXP);
    }

    @Override
    public float getYaw() {
        return super.getYaw() - (float)gyroOffset;
    }

    public void setYaw(double yawAngle) {
        gyroOffset = super.getYaw() - yawAngle;
    }
}