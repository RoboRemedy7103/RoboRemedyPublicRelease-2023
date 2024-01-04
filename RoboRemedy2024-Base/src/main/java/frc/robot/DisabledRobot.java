// DisabledRobot.java - Code for disabled mode
package frc.robot;

import edu.wpi.first.wpilibj.*;
import frc.robot.RobotEnums.*;

public class DisabledRobot {
    private Electronics e;
    private RoboLog rLog;
    private Action act;
    private OI oi;
    private RobotState robotState;
    private Dashboard dash;

    private AutoProgram lastAuto = AutoProgram.None;

    public DisabledRobot(Electronics e, RoboLog rLog, Action act, OI oi,
            RobotState robotState, Dashboard dash) {
        this.e = e;
        this.rLog = rLog;
        this.act = act;
        this.oi = oi;
        this.robotState = robotState;
        this.dash = dash;

        if (this.rLog == null) {
            System.out.println("Warning: DisabledRobot.rLog is null");
        }
        if (this.e == null) {
            rLog.print("Warning: DisabledRobot.e is null");
        }
        if (this.act == null) {
            rLog.print("Warning: DisabledRobot.act is null");
        }
        if (this.oi == null) {
            rLog.print("Warning: DisabledRobot.oi is null");
        }
        if (this.robotState == null) {
            rLog.print("Warning: DisabledRobot.robotState is null");
        }
        if (this.dash == null) {
            rLog.print("Warning: DisabledRobot.dash is null");
        }
    }

    public void disabledInit() {
        PhotonVision.setCubeCameraDriverMode(false);
    }

    public void disabledPeriodic() {
        if (lastAuto != dash.getAutoMode()) {
            lastAuto = dash.getAutoMode();
            rLog.print("Auto Selection = " + lastAuto);
        }
        if (DriverStation.isDSAttached() == false) {
            if (PhotonVision.isAprilTagLeftCameraAttached() == false) {
                e.setLED(LEDregion.regionOne, LEDcolor.DimRed);
            } else {
                e.setLED(LEDregion.regionOne, LEDcolor.DimWhite);
            }
            if (PhotonVision.isAprilTagRightCameraAttached() == false) {
                e.setLED(LEDregion.regionTwo, LEDcolor.DimRed);
            } else {
                e.setLED(LEDregion.regionTwo, LEDcolor.DimWhite);
            }
        } else {
            if (PhotonVision.isAprilTagLeftCameraAttached() == false) {
                e.setLED(LEDregion.regionOne, LEDcolor.DimRed);
            } else {
                e.setLED(LEDregion.regionOne, LEDcolor.DimBlue);
            }
            if (PhotonVision.isAprilTagRightCameraAttached() == false) {
                e.setLED(LEDregion.regionTwo, LEDcolor.DimRed);
            } else {
                e.setLED(LEDregion.regionTwo, LEDcolor.DimBlue);
            }
        }
    }
}