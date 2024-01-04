// RobotState.java - Place to store info about the robot so they can be calculated once and used multiple times
package frc.robot;

import edu.wpi.first.wpilibj.*;

public class RobotState {
    private RoboLog rLog;

    public boolean isAprilTagFound = false;
    public int aprilTagId = 0;
    public double aprilTagInchesInFrontOfTarget;
    public double aprilTagInchesRightOfTarget;
    public boolean distanceSensorValid = false;
    public double distanceSensorInches = 0;
    public boolean logAprilTags = true;
    public boolean isLidarConnected = false;

    public AddressableLED ledString;
    public AddressableLEDBuffer ledBuffer;

    public RobotState(RoboLog rLog) {
        this.rLog = rLog;

        if (this.rLog == null) {
            System.out.println("Warning: RobotState.rLog is null");
        }
    }

    public void stopLoggingAprilTags() {
        logAprilTags = false;
    }

    public void startLoggingAprilTags() {
        logAprilTags = true;
        isAprilTagFound = false;
    }

}