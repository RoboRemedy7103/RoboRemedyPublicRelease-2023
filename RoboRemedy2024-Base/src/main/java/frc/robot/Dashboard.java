// Dashboard.java - Controls the Shuffleboard dashboard
package frc.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.networktables.*;
import frc.robot.RobotEnums.*;
import java.util.Map;

public class Dashboard {
    private RoboLog rLog;
    private Electronics e;
    private RobotState robotState;
    private Action act;
    private OI oi;

    // Teleop Tab
    private GenericEntry alliance;
    private GenericEntry teleopTime = null;

    // Test Tab
    private GenericEntry testString = null;
    private SendableChooser<TestMode> testChooser = null;
    private GenericEntry demoMode = null;
    private GenericEntry allCamerasWork = null;
    private GenericEntry distanceSensor = null;
    private GenericEntry testTime = null;
    private GenericEntry isAprilTagFound = null;
    private GenericEntry aprilTagId = null;
    private GenericEntry aprilTagInchesInFrontOfTarget;
    private GenericEntry aprilTagInchesRightOfTarget;
    private GenericEntry driverStick = null;
    private GenericEntry operatorStick = null;
    private GenericEntry isLidarConnected = null;

    // Autonomous Tab
    private SendableChooser<AutoProgram> autoChooser = null;
    private GenericEntry autoValue = null;
    private GenericEntry autoTime = null;

    // PreGame Tab
    private GenericEntry frontLeftDrive;
    private GenericEntry frontLeftTurn;
    private GenericEntry frontRightDrive;
    private GenericEntry frontRightTurn;
    private GenericEntry backLeftDrive;
    private GenericEntry backLeftTurn;
    private GenericEntry backRightDrive;
    private GenericEntry backRightTurn;
    private GenericEntry isBothJoystick;
    private GenericEntry isBattery;
    private GenericEntry isAprilTagRightCameraAttached;
    private GenericEntry isAprilTagLeftCameraAttached;
    private GenericEntry isCubeCameraAttached;

    Dashboard(Electronics e, RoboLog rLog, RobotState robotState, Action act, OI oi) {
        this.e = e;
        this.rLog = rLog;
        this.robotState = robotState;
        this.act = act;
        this.oi = oi;

        if (this.rLog == null) {
            System.out.println("Warning: Dashboard.rLog is null");
        }
        if (this.robotState == null) {
            rLog.print("Warning: Dashboard.robotState is null");
        }
        if (this.act == null) {
            rLog.print("Warning: Dashboard.act is null");
        }
        if (this.e == null) {
            rLog.print("Warning: Dashboard.e is null");
        }
        if (this.oi == null) {
            rLog.print("Warning: Dashboard.oi is null");
        }

        InitTelop();
        InitTest();
        InitAutonomous();
        InitPreGame();
    }

    public void selectTab(String selectTab) {
        rLog.print("Selecting new tab: " + selectTab);
        Shuffleboard.selectTab(selectTab);
    }

    public void InitTelop() {
        Shuffleboard.getTab("Teleop")
            .add("Robot Name", e.getRobotName())
            .withPosition(6, 0)
            .getEntry();

        demoMode = Shuffleboard.getTab("Teleop")
            .add("Demo Mode", false)
            .withWidget(BuiltInWidgets.kToggleButton)
            .withPosition(0, 0)
            .getEntry();

        Shuffleboard.getTab("Teleop")
            .add("Gyro", e.gyro)
            .withWidget(BuiltInWidgets.kGyro)
            .withPosition(7, 0)
            .withSize(2, 2);

        alliance = Shuffleboard.getTab("Teleop")
            .add("Alliance", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withProperties(Map.of("color when true", "red", "color when false", "blue"))
            .withPosition(1, 2)
            .withSize(4, 1)
            .getEntry();

        teleopTime = Shuffleboard.getTab("Teleop")
            .add("Time", "")
            .withPosition(7, 2)
            .withSize(2, 1)
            .getEntry();

        driverStick = Shuffleboard.getTab("Teleop")
            .add("Driver Joystick", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withProperties(Map.of("color when true", "green", "color when false", "white"))
            .withPosition(3, 1)
            .withSize(1, 1)
            .getEntry();
        
        operatorStick = Shuffleboard.getTab("Teleop")
            .add("Operator Joystick", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withProperties(Map.of("color when true", "green", "color when false", "white"))
            .withPosition(4, 1)
            .withSize(1, 1)
            .getEntry();
    }

    public void InitTest() {
        testChooser = new SendableChooser<TestMode>();
        testChooser.setDefaultOption("Quick Test", TestMode.QuickTest);
        testChooser.addOption("Swerve Test", TestMode.SwerveTest);
        testChooser.addOption("Swerve Drive", TestMode.SwerveDrive);
        testChooser.addOption("Test Angle", TestMode.TestAngle);
        testChooser.addOption("Test Motors", TestMode.TestMotors);
        testChooser.addOption("Tune Motor", TestMode.TuneMotor);
        testChooser.addOption("Test None", TestMode.TestNone);
        testChooser.addOption("Test LED", TestMode.TestLED);
        testChooser.addOption("Reset Encoders", TestMode.ResetEncoders);
        testChooser.addOption("Test Countdown", TestMode.TestCountdown);

        Shuffleboard.getTab("Test")
            .add("Test Mode", testChooser)
            .withWidget(BuiltInWidgets.kComboBoxChooser)
            .withPosition(0, 0)
            .withSize(2, 1);

        Shuffleboard.getTab("Test")
            .add("Gyro Deg", e.gyro)
            .withWidget(BuiltInWidgets.kGyro)
            .withPosition(7, 0)
            .withSize(2, 2);

        Shuffleboard.getTab("Test")
            .add("Robot Name", e.getRobotName())
            .withPosition(6, 0)
            .withSize(1, 1)
            .getEntry();

        testTime = Shuffleboard.getTab("Test")
            .add("Time", "")
            .withPosition(7, 2)
            .withSize(2, 1)
            .getEntry();

        distanceSensor = Shuffleboard.getTab("Test")
            .add("Distance Sensor", "")
            .withPosition(5, 0)
            .withSize(1, 1)
            .getEntry();

        testString = Shuffleboard.getTab("Test")
            .add("Test String", "")
            .withPosition(0, 1)
            .withSize(2, 1)
            .getEntry();

        allCamerasWork = Shuffleboard.getTab("Test")
            .add("All Cams", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withProperties(Map.of("color when true", "green", "color when false", "red"))
            .withPosition(3,2)
            .getEntry();

        isAprilTagFound = Shuffleboard.getTab("Test")
            .add("April Tag Found", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withProperties(Map.of("color when true", "green", "color when false", "red"))
            .withPosition(1,2)
            .getEntry();
        
        aprilTagId = Shuffleboard.getTab("Test")
            .add("April Tag Id", "")
            .withPosition(2, 2)
            .withSize(1, 1)
            .getEntry();

        aprilTagInchesInFrontOfTarget = Shuffleboard.getTab("Test")
            .add("Inches In Front Of Target", "")
            .withPosition(4, 2)
            .withSize(1, 1)
            .getEntry();

        aprilTagInchesRightOfTarget = Shuffleboard.getTab("Test")
            .add("Inches Right Of Target", "")
            .withPosition(5, 2)
            .withSize(1, 1)
            .getEntry();
        }

    public void InitAutonomous() {
        autoChooser = new SendableChooser<AutoProgram>();
        autoChooser.setDefaultOption("None", AutoProgram.None);
        autoChooser.addOption("Just Drive Out", AutoProgram.DriveOut);
        autoChooser.addOption("Just Align", AutoProgram.JustAlign);
        
        Shuffleboard.getTab("Autonomous")
            .add("Autonomous Mode", autoChooser)
            .withWidget(BuiltInWidgets.kComboBoxChooser)
            .withPosition(0, 0)
            .withSize(2, 1);
        autoValue = Shuffleboard.getTab("Autonomous")
            .add("Auto Value", 0.0)
            .withPosition(0, 1)
            .withSize(1, 1)
            .getEntry();

        Shuffleboard.getTab("Autonomous")
            .add("Robot Name", e.getRobotName())
            .withPosition(6, 0)
            .withSize(1, 1)
            .getEntry();

        Shuffleboard.getTab("Autonomous")
            .add("Gyro", e.gyro)
            .withWidget(BuiltInWidgets.kGyro)
            .withPosition(7, 0)
            .withSize(2, 2);

        autoTime = Shuffleboard.getTab("Autonomous")
            .add("Time", "")
            .withPosition(7, 2)
            .withSize(2, 1)
            .getEntry();
    }

    public void InitPreGame() {
        frontLeftDrive = Shuffleboard.getTab("PreGame")
            .add("fL Drive", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(0, 0)
            .getEntry();

        frontLeftTurn = Shuffleboard.getTab("PreGame")
            .add("fL Turn", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(0, 1)
            .getEntry();

        frontRightDrive = Shuffleboard.getTab("PreGame")
            .add("fR Drive", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(1, 0)
            .getEntry();

        frontRightTurn = Shuffleboard.getTab("PreGame")
            .add("fR Turn", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(1, 1)
            .getEntry();

        backLeftDrive = Shuffleboard.getTab("PreGame")
            .add("bL Drive", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(2, 0)
            .getEntry();

        backLeftTurn = Shuffleboard.getTab("PreGame")
            .add("bL Turn", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(2, 1)
            .getEntry();

        backRightDrive = Shuffleboard.getTab("PreGame")
            .add("bR Drive", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(3, 0)
            .getEntry();

        backRightTurn = Shuffleboard.getTab("PreGame")
            .add("bR Turn", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(3, 1)
            .getEntry();

        Shuffleboard.getTab("PreGame")
            .add("Robot Name", e.getRobotName())
            .withPosition(6, 0)
            .getEntry();

        Shuffleboard.getTab("PreGame")
            .add("Gyro Deg", e.gyro)
            .withPosition(7, 0)
            .withWidget(BuiltInWidgets.kGyro)// kDial
            .withSize(2, 2);

        isBothJoystick = Shuffleboard.getTab("PreGame")
            .add("Both Joystick", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withProperties(Map.of("color when true", "green", "color when false", "red"))
            .withPosition(8, 2)
            .getEntry();

        isBattery = Shuffleboard.getTab("PreGame")
            .add("Battery OK", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withProperties(Map.of("color when true", "green", "color when false", "red"))
            .withPosition(6, 2)
            .getEntry();
        
        isAprilTagRightCameraAttached = Shuffleboard.getTab("PreGame")
            .add("RightCam", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withProperties(Map.of("color when true", "green", "color when false", "red"))
            .withPosition(5, 2)
            .getEntry();
        
        isAprilTagLeftCameraAttached = Shuffleboard.getTab("PreGame")
            .add("LeftCam", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withProperties(Map.of("color when true", "green", "color when false", "red"))
            .withPosition(4, 2)
            .getEntry();
            
        isCubeCameraAttached = Shuffleboard.getTab("PreGame")
            .add("CubeCam", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withProperties(Map.of("color when true", "green", "color when false", "red"))
            .withPosition(2, 2)
            .getEntry();

        isLidarConnected = Shuffleboard.getTab("PreGame")
            .add("Lidar Camera", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withProperties(Map.of("color when true", "green", "color when false", "red"))
            .withPosition(0, 2)
            .getEntry();
    }

    public void dashboardPeriodic() {
        // Test Entries
        setDistanceInInches(RobotMath.round2(robotState.distanceSensorInches));
        setRoboTime(DriverStation.getMatchTime());
        isAprilTagFound.setBoolean(robotState.isAprilTagFound);
        if (robotState.isAprilTagFound) {
            aprilTagId.setString(String.valueOf(robotState.aprilTagId));
            aprilTagInchesInFrontOfTarget
                    .setString(String.valueOf(RobotMath.round2(robotState.aprilTagInchesInFrontOfTarget)));
            aprilTagInchesRightOfTarget
                    .setString(String.valueOf(RobotMath.round2(robotState.aprilTagInchesRightOfTarget)));
        } else {
            aprilTagId.setString("");
            aprilTagInchesInFrontOfTarget.setString("");
            aprilTagInchesRightOfTarget.setString("");
        }
    
        // Teleop Entries
        alliance.setBoolean(e.isRedAlliance());
        driverStick.setBoolean(oi.isJoystickPovPressed(0, 0));
        operatorStick.setBoolean(oi.isJoystickPovPressed(1, 0));
        // Autonomous Entries
        // Pre-Game Entries 
        isAprilTagRightCameraAttached.setBoolean(PhotonVision.isAprilTagRightCameraAttached());
        isAprilTagLeftCameraAttached.setBoolean(PhotonVision.isAprilTagLeftCameraAttached());
        isCubeCameraAttached.setBoolean(PhotonVision.isCubeCameraAttached());
        isLidarConnected.setBoolean(robotState.isLidarConnected);
        // Swerve
        frontLeftDrive.setBoolean(e.isMotorAttached(true, 0, 2, 0));
        frontLeftTurn.setBoolean(e.isMotorAttached(true, 0, 1, 0));
        frontRightDrive.setBoolean(e.isMotorAttached(true, 1, 2, 0));
        frontRightTurn.setBoolean(e.isMotorAttached(true, 1, 1, 0));
        backLeftDrive.setBoolean(e.isMotorAttached(true, 3, 2, 0));
        backLeftTurn.setBoolean(e.isMotorAttached(true, 3, 1, 0));
        backRightDrive.setBoolean(e.isMotorAttached(true, 2, 2, 0));
        backRightTurn.setBoolean(e.isMotorAttached(true, 2, 1, 0));
        // others
        isBothJoystick.setBoolean(DriverStation.isJoystickConnected(1) && DriverStation.isJoystickConnected(0)
                && DriverStation.getJoystickIsXbox(0) && DriverStation.getJoystickIsXbox(1));
        isBattery.setBoolean(e.isBatteryGood());
        allCamerasWork.setBoolean(PhotonVision.isAprilTagLeftCameraAttached() && PhotonVision.isAprilTagRightCameraAttached()
                && PhotonVision.isCubeCameraAttached());
    }

    public TestMode getTestMode() {
        if (testChooser != null)
            return testChooser.getSelected();
        else
            return TestMode.TestNone;
    }

    public AutoProgram getAutoMode() {
        if (autoChooser != null)
            return autoChooser.getSelected();
        else
            return AutoProgram.JustAlign;
    }

    public double getAutoValue() {
        if (autoValue != null)
            return autoValue.getDouble(0.0);
        else
            return 0.0;
    }

    public void setTestString(String string) {
        if (testString != null)
            testString.setString(string);
    }

    public void setDistanceInInches(double dist) {
        if (distanceSensor != null) {
            distanceSensor.setString(String.valueOf(dist));
        }
    }

    public void setRoboTime(double time) {
        if (teleopTime != null) {
            teleopTime.setString(String.valueOf(RobotMath.round1((time))));
        } if (testTime != null) {
            testTime.setString(String.valueOf(RobotMath.round1((time))));
        } if (autoTime != null) {
            autoTime.setString(String.valueOf(RobotMath.round1((time))));
        }
    }

    public boolean isDemoMode() {
        if (demoMode != null)
            return demoMode.getBoolean(false);
        else
            return false;
    }
}