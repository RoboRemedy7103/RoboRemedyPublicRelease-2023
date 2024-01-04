// RobotPeriodic.java - Code for robot periodic code (all modes)
package frc.robot;

import edu.wpi.first.wpilibj.*;

public class RobotPeriodic {
    private Electronics e;
    private RoboLog rLog;
    private Action act;
    private OI oi;
    private RobotState robotState;
    private Dashboard dash;

    public RobotPeriodic(Electronics e, RoboLog rLog, Action act, OI oi,
            RobotState robotState, Dashboard dash) {
        this.e = e;
        this.rLog = rLog;
        this.act = act;
        this.oi = oi;
        this.robotState = robotState;
        this.dash = dash;

        if (this.rLog == null) {
            System.out.println("Warning: RobotPeriodic.rLog is null");
        }
        if (this.e == null) {
            rLog.print("Warning: RobotPeriodic.e is null");
        }
        if (this.act == null) {
            rLog.print("Warning: RobotPeriodic.act is null");
        }
        if (this.oi == null) {
            rLog.print("Warning: RobotPeriodic.oi is null");
        }
        if (this.robotState == null) {
            rLog.print("Warning: RobotPeriodic.robotState is null");
        }
        if (this.dash == null) {
            rLog.print("Warning: RobotPeriodic.dash is null");
        }
    }

    public void robotPeriodic() {
        if (DriverStation.isDSAttached()) {
            dash.dashboardPeriodic();
            if (oi.getDriverResetGyroButtonPressed()) {
                e.resetGyro();
                e.resetTilt();
            } else if (oi.getDriverReverseResetGyroButtonPressed()) {
                e.setGyro(180);
                e.resetTilt();
            }
        }

        e.sendLEDBuffer();
        
        e.robotPeriodic();

        robotState.distanceSensorValid = e.isDistanceSensorRangeValid();
        robotState.distanceSensorInches = e.getDistanceInInches();

        PhotonVision.TargetInfo info =
            PhotonVision.getBestAprilTagForCamera(0, 0, 0);
        robotState.isAprilTagFound = info.isFound;
        robotState.aprilTagId  = info.id;
        robotState.aprilTagInchesInFrontOfTarget = info.inchesInFrontOfTarget;
        robotState.aprilTagInchesRightOfTarget = info.inchesRightOfTarget;

        robotState.isLidarConnected = e.isLidarConnected();

        act.logChanges();
    }
}