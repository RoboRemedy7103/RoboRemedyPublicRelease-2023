package frc.robot;

import edu.wpi.first.wpilibj.*;

import edu.wpi.first.networktables.*;

public class LidarCamera {
    long lastCounter = 0;

    Timer changeTime = new Timer();

    final double TIME_OUT = 0.5;

    NetworkTable networkTable;
    DoublePublisher regionXPub;
    DoublePublisher regionYPub;

    DoubleSubscriber zSubscriber;
    IntegerSubscriber counterSubscriber;

    public LidarCamera(String networkTableName) {
        networkTable = NetworkTableInstance.getDefault().getTable(networkTableName);
        regionXPub = networkTable.getDoubleTopic("regionX").publish();
        regionYPub = networkTable.getDoubleTopic("regionY").publish();

        zSubscriber = networkTable.getDoubleTopic("z").subscribe(-1.0);
        counterSubscriber = networkTable.getIntegerTopic("counter").subscribe(-1);

        changeTime.reset();
        changeTime.start();
    }

    public void setRegion(double regionX, double regionY) {
        regionXPub.set(regionX);
        regionYPub.set(regionY);
    }

    public double getRegionZ() {
        return zSubscriber.get();
    }

    public boolean isConnected() {
        long counterValue = counterSubscriber.get();
        if (counterValue == -1)
            return false;

        double currentTime = changeTime.get();
        if (counterValue != lastCounter) {
            lastCounter = counterValue;
            changeTime.reset();
            changeTime.start();
            return true;
        } else {
            if (currentTime <= TIME_OUT) {
                return true;
            } else {
                return false;
            }
        }
    }
}
