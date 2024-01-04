// Robot.java - Main robot code for 2024
package frc.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotEnums.*;

public class Robot extends TimedRobot {

    private final String projectName = "2024 Competition";
    private RoboLog rLog = new RoboLog();
    private RobotState robotState = new RobotState(rLog);
    private Electronics e = new Electronics(true, rLog, robotState);
    private OI oi = new OI();
    private Action act = new Action(e, rLog, robotState, oi);;
    private Dashboard dash = new Dashboard(e, rLog, robotState, act, oi);
    private AutoRobot auto = new AutoRobot(e, rLog, act, dash, robotState);
    private TeleopRobot teleop = new TeleopRobot(e, rLog, act, oi, robotState, dash);
    private TestRobot test = new TestRobot(e, rLog, act, dash, robotState, oi);
    private RobotPeriodic periodic = new RobotPeriodic(e, rLog, act, oi, robotState, dash);
    private DisabledRobot disabled = new DisabledRobot(e, rLog, act, oi, robotState, dash);
    private Timer autoTimer = new Timer();

    /* Called once when robot starts */
    @Override
    public void robotInit() {
        rLog.print(projectName + " robotInit");
        e.assignAllEncoderValues();
        LiveWindow.disableAllTelemetry(); // Improve performance
        SmartDashboard.updateValues(); // Improve performance
        PhotonVision.PhotonVisionInit();
        e.setSwerveCoast();
        e.resetTilt();
    }

    /* Called periodically in all modes */
    @Override
    public void robotPeriodic() {
        periodic.robotPeriodic();
    }

    // Called once whenever robot is disabled
    @Override
    public void disabledInit() {
        rLog.setRobotMode("DISA", "Disabled");
        disabled.disabledInit();
    }
    /* Called periodically while robot is disabled */
    @Override
    public void disabledPeriodic() {
        disabled.disabledPeriodic();
    }

    /* Called once when autonomous is started */
    @Override
    public void autonomousInit() {
        rLog.setRobotMode("AUTO", "Autonomus");
        AutoProgram autoSelected = dash.getAutoMode();
        auto.autonomousInit(autoSelected);
    }

    /* Called periodically during autonomous */
    @Override
    public void autonomousPeriodic() {
        auto.autonomousPeriodic();
        autoTimer.reset();
        autoTimer.start();
    }

    /* Called once when operator control is started */
    @Override
    public void teleopInit() {
        rLog.setRobotMode("TELE", "Teleop");
        rLog.print(projectName + " teleopInit");
        teleop.teleopInit();
    }

    /* Called periodically during operator control */
    @Override
    public void teleopPeriodic() {
        teleop.teleopPeriodic();
    }

    /* Called once when test mode is started */
    @Override
    public void testInit() {
        rLog.setRobotMode("TEST", "Test");
        rLog.print(projectName + " testInit");
        test.testInit();
    }

    /* Called periodically during test mode */
    @Override
    public void testPeriodic() {
        test.testPeriodic();
    }
}