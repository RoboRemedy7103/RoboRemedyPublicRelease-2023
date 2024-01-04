// AutoRobot.java - Code for autonomous programs
package frc.robot;

import edu.wpi.first.wpilibj.*;
import frc.robot.RobotEnums.*;

public class AutoRobot {
    private Electronics e;
    private RoboLog rLog;
    private Action act;
    private Dashboard dash;
    private RobotState robotState;

    private int stepNumber;
    private Timer timer1 = new Timer();
    private AutoProgram autoSelection;

    private static final double MAX_ACCEL = 100.0;

    public AutoRobot(Electronics e, RoboLog rLog, Action act, Dashboard d, RobotState robotState) {
        this.e = e;
        this.rLog = rLog;
        this.act = act;
        this.dash = d;
        this.robotState = robotState;

        if (this.rLog == null) {
            System.out.println("Warning: AutoRobot.rLog is null");
        }
        if (this.dash == null) {
            rLog.print("Warning: AutoRobot.dash is null");
        }
        if (this.robotState == null) {
            rLog.print("Warning: AutoRobot.robotState is null");
        }
        if (this.e == null) {
            rLog.print("Warning: AutoRobot.e is null");
        }
        if (this.act == null) {
            rLog.print("Warning: AutoRobot.act is null");
        }
    }

    public void autonomousInit(AutoProgram autoSelected) {
        rLog.print("Start Autonomous Program: " + autoSelected);
        robotState.startLoggingAprilTags();
        e.resetGyro();
        e.resetTilt();
        this.autoSelection = autoSelected;
        act.actionReset();
        e.stopSwerveMotors();
        setStepNumber(1);
        e.stopAllMotors();
        e.setGyro(0);
        PhotonVision.setCubeCameraDriverMode(false);
    }

    public void autonomousPeriodic() {
        if (autoSelection == AutoProgram.JustAlign) {
            e.alignSwerveMotorsForward();
        } else if (autoSelection == AutoProgram.DriveOut) {
            DriveOut();
        } else if (autoSelection == AutoProgram.None) {
            e.stopAllMotors();
        }

        if (stepNumber % 4 == 0) {
            e.setLED(LEDregion.regionOne, LEDcolor.Yellow);
        } else if (stepNumber % 4 == 1) {
            e.setLED(LEDregion.regionOne, LEDcolor.Green);
        } else if (stepNumber % 4 == 2) {
            e.setLED(LEDregion.regionOne, LEDcolor.White);
        } else if (stepNumber % 4 == 3) {
            e.setLED(LEDregion.regionOne, LEDcolor.Purple);
        }
    }

    void setStepNumber(int number) {
        stepNumber = number;
        act.actionReset();
        timer1.reset();
        timer1.start();
        rLog.print("New Auto Step Number: " + stepNumber + " Gyro: " + RobotMath.round1(e.getGyro()));
    }

    void setNextStepNumber() {
        setStepNumber(stepNumber + 1);
    }

    void DriveOut() {
        switch (stepNumber) {
            case 1: // Drive Out: Drive forward 24 inches
                if (act.driveStraightWithFacing(0.0, 40.0,
                        180.0, MAX_ACCEL,
                        24.0, 0.0)) {
                    setNextStepNumber();
                }
                break;
            case 2: // Drive Out: Stop
                e.stopAllMotors();
                break;
        }
    }
}