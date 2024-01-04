// RobotMotor.java - Class to be used for any type of motor
package frc.robot;

import com.ctre.phoenix6.*;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.*;

public class RobotCANcoder {

    CANcoder canCoder;
    RoboLog rLog;

    RobotCANcoder(int id, boolean isInverted, RoboLog rLog) {
        this.rLog = rLog;
        canCoder = new CANcoder(id);
        MagnetSensorConfigs magnetConfig = new MagnetSensorConfigs();
        canCoder.getConfigurator().refresh(magnetConfig);
        magnetConfig.SensorDirection = (isInverted ? SensorDirectionValue.Clockwise_Positive :
            SensorDirectionValue.CounterClockwise_Positive);
        magnetConfig.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        canCoder.getConfigurator().apply(magnetConfig);
    }

    double getAbsoluteAngle() {
        StatusSignal<Double> position = canCoder.getAbsolutePosition();
        return position.getValue() * 360.0;
    }

    void setAbsoluteAngle(double angle) {
        MagnetSensorConfigs magnetConfig = new MagnetSensorConfigs();
        canCoder.getConfigurator().refresh(magnetConfig);
        double magnetOffset = magnetConfig.MagnetOffset;
        double currentEncoderValue = canCoder.getAbsolutePosition().getValue();
        magnetConfig.MagnetOffset = (magnetOffset + (angle / 360.0) - currentEncoderValue) % 1.0;
        rLog.print("Magnet offset changed from " + RobotMath.round2(magnetOffset) + " to " +
            RobotMath.round2(magnetConfig.MagnetOffset));
        canCoder.getConfigurator().apply(magnetConfig);
    }

    double getAngle() {
        StatusSignal<Double> position = canCoder.getPosition();
        return position.getValue() * 360.0;
    }

    void setAngle(double angle) {
        canCoder.setPosition(angle / 360.0);
    }
}