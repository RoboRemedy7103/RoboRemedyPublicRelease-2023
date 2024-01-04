// Electronics.java - Controls the motors, gyro, encoders, solenoids, etc.
package frc.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.DriverStation.*;
import edu.wpi.first.wpilibj.util.*;
import frc.robot.LEDCountdown.SingleLED;
import frc.robot.RobotMotor.*;
import frc.robot.RobotEnums.*;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import java.io.File;
import java.util.Optional;

public class Electronics {
    private static final double DISTANCE_SENSOR_OFFSET = 0.0;
    private static LinearMapper swerveVelocityMapper = new LinearMapper();
    private RobotName robotName;

    private RoboLog rLog;
    private RobotState robotState;

    private SwerveModule frontLeft;
    private SwerveModule frontRight;
    private SwerveModule backRight;
    private SwerveModule backLeft;
    private SwerveModule[] swerveDrives;
    private SwerveState currentState;
    private SwerveState centerPivotState;
    public RobotGyro gyro = new RobotGyro(); // public for the dashboard
    private double lastTravelVelocity = 0;
    private Timer commandTimer = new Timer();
    private Timer batteryTimer = new Timer();
    private double batteryVoltage = 0;
    private double lastCommandTime = 0;
    private double maxTeleopInchesPerSecond = 0;
    private TimeOfFlight distSensor;
    private PowerDistribution pdp;
    private LidarCamera lidar;

    private double lastTravelAngleDegrees = 0.0;
    private double lastTravelRotationDegreesPerSecond = 0.0;
    private boolean lastYellowButtonsValue = false;
    private double pitchOffset = 0.0;
    private double rollOffset = 0.0;

    // Button
    private DigitalInput yellowButton1;
    private DigitalInput yellowButton2;

    private int ledLength = 250;
    private AddressableLED ledString = new AddressableLED(0); // port 0
    private AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(ledLength);
    private Color rgb;
    private double LEDStrobe;
    private LEDCountdown timerStrip = new LEDCountdown();
    
    //Used for setting up LED strips for RGB
    public class RGB {
        int startingLED;
        int endingLED;
        int currentLED = startingLED;
        int segmentLength = endingLED - startingLED;
        int ledLength;
        short changeSpeed;
        
        private AddressableLED rgbString;
        private AddressableLEDBuffer rgbBuffer;
        private Timer rgbTimer = new Timer();
        
        short redValue = 0;
        short greenValue = 0;
        short blueValue = 255;
        byte phase = 1;
        boolean ledToggle = false;
        boolean toggleDone = false;

        private void advanceColor() {
            switch (phase) {
                case 1:
                    if (redValue == 255) {
                        phase = 2;
                    } else {
                        redValue += 1;
                        blueValue -= 1;
                    }
                    break;
                case 2:
                    if (greenValue == 255) {
                        phase = 3;
                    } else {
                        greenValue += 1;
                        redValue -= 1;
                    }
                    break;
                case 3:
                    if (blueValue == 255) {
                        phase = 1;
                    } else {
                        blueValue += 1;
                        greenValue -= 1;
                    }
                    break;
            }
        }

        private void revertColor() {
            switch (phase) {
                case 1:
                    if (blueValue == 255) {
                        phase = 3;
                    } else {
                        redValue -= 1;
                        blueValue += 1;
                    }
                    break;
                case 2:
                    if (redValue == 255) {
                        phase = 1;
                    } else {
                        greenValue -= 1;
                        redValue += 1;
                    }
                    break;
                case 3:
                    if (greenValue == 255) {
                        phase = 2;
                    } else {
                        blueValue -= 1;
                        greenValue += 1;
                    }
                    break;
            }
        }
        
        void rgbToggle (boolean input) {
            if (input && !toggleDone) {
                rgbTimer.start();
                if (ledToggle)
                    ledToggle = false;
                else
                    ledToggle = true;
                toggleDone = true;
            } else if (!input && toggleDone) {
                toggleDone = false;
            }
        }

        void smoothRGBActivate() {
            if (ledToggle) {
                    for (currentLED = startingLED; currentLED <= endingLED; currentLED++) {
                        rgbBuffer.setRGB(currentLED, redValue, greenValue, blueValue);
                        for (short reps = changeSpeed; reps >= 0; reps--) {
                            advanceColor();
                        }
                    }
                    for (currentLED = endingLED; currentLED >= startingLED + 1; currentLED--) {
                        for (short reps = changeSpeed; reps >= 0; reps--) {
                            revertColor();
                        }
                    }
            } else {
                for (currentLED = startingLED; currentLED <= endingLED; currentLED++) {
                    rgbBuffer.setRGB(currentLED, 0, 0, 0);
                }
            }
        }

        void solidRGBActivate() {
            if (ledToggle) {
                for (currentLED = startingLED; currentLED <= endingLED; currentLED++) {
                    rgbBuffer.setRGB(currentLED, redValue, greenValue, blueValue);
                    for (short reps = changeSpeed; reps >= 0; reps--) {
                        advanceColor();
                    }
                }
            }
        }
    }

    Electronics(boolean fullSwerve, RoboLog rLog, RobotState robotState) {
        this.robotState = robotState;
        this.rLog = rLog;

        if (this.rLog == null) {
            System.out.println("Warning: Electronics.rLog is null");
        }
        if (this.robotState == null) {
            rLog.print("Warning: Electronics.robotState is null");
        }

        File jr = new File("/home/lvuser/jr.txt");
        File invisi = new File("/home/lvuser/scorpio.txt");

        if (jr.isFile()) {
            rLog.print("Robot = Jerry Junior");
            robotName = RobotName.JerryJr;
        } else if (invisi.isFile()) {
            rLog.print("Robot = Scorpio");
            robotName = RobotName.Scorpio;
        } else {
            rLog.print("Robot = Live 2024");
            robotName = RobotName.LIVE2024;
        }

        if (robotName == RobotName.JerryJr) {
            centerPivotState = new SwerveState(13.5, 13.5);

            currentState = centerPivotState;
            distSensor = null; // new TimeOfFlight(2);

            frontLeft = new SwerveModule(41, 51, 01, 0, rLog, RobotMotorType.TalonFX, RobotMotorType.SparkMax,
                    RobotEncoderType.Cancoder, 3.77, 8.308, 20);
            frontRight = new SwerveModule(42, 52, 2, 0, rLog, RobotMotorType.TalonFX, RobotMotorType.SparkMax,
                    RobotEncoderType.Cancoder, 3.77, 8.308, 20);
            backRight = new SwerveModule(43, 53, 3, 0, rLog, RobotMotorType.TalonFX, RobotMotorType.SparkMax,
                    RobotEncoderType.Cancoder, 3.77, 8.308, 20);
            backLeft = new SwerveModule(44, 54, 4, 0, rLog, RobotMotorType.TalonFX, RobotMotorType.SparkMax,
                    RobotEncoderType.Cancoder, 3.77, 8.308, 20);

            maxTeleopInchesPerSecond = 77.0;

            pdp = new PowerDistribution();
            yellowButton1 = null;
            yellowButton2 = null;
        } else if (robotName == RobotName.Scorpio) {
            centerPivotState = new SwerveState(17.6, 23.5);

            currentState = centerPivotState;

            swerveVelocityMapper.add(0.8, 0.027);
            swerveVelocityMapper.add(3.7, 0.035);
            swerveVelocityMapper.add(13.6, 0.067);
            swerveVelocityMapper.add(20.8, 0.1);
            swerveVelocityMapper.add(25.2, 0.119);
            swerveVelocityMapper.add(62.6, 0.3);
            swerveVelocityMapper.add(83.8, 0.399);
            swerveVelocityMapper.add(93.9, 0.45);
            swerveVelocityMapper.add(167.9, 0.8);
            swerveVelocityMapper.add(187.7, 0.9);
            swerveVelocityMapper.add(195.9, 0.94);

            // Swerves
            frontLeft = new SwerveModule(41, 51, 01, 0, rLog, RobotMotorType.TalonFX, RobotMotorType.TalonFX,
                    RobotEncoderType.Cancoder, 3.91, 6.12, 28.14);
            frontRight = new SwerveModule(42, 52, 2, 0, rLog, RobotMotorType.TalonFX, RobotMotorType.TalonFX,
                    RobotEncoderType.Cancoder, 3.91, 6.12, 28.14);
            backRight = new SwerveModule(43, 53, 3, 0, rLog, RobotMotorType.TalonFX, RobotMotorType.TalonFX,
                    RobotEncoderType.Cancoder, 3.91, 6.12, 28.14);
            backLeft = new SwerveModule(44, 54, 4, 0, rLog, RobotMotorType.TalonFX, RobotMotorType.TalonFX,
                    RobotEncoderType.Cancoder, 3.91, 6.12, 28.14);

            frontLeft.setVelocityMapper(swerveVelocityMapper);
            frontRight.setVelocityMapper(swerveVelocityMapper);
            backRight.setVelocityMapper(swerveVelocityMapper);
            backLeft.setVelocityMapper(swerveVelocityMapper);
        
            maxTeleopInchesPerSecond = 130.0;

            pdp = new PowerDistribution();
            distSensor = null; // new TimeOfFlight(2);
            yellowButton1 = new DigitalInput(3);
            yellowButton2 = new DigitalInput(4);

            timerStrip.add(0, 1);
            timerStrip.add(1, 2);
            timerStrip.add(2, 3);
            timerStrip.add(3, 4);
            timerStrip.add(4, 5);
            timerStrip.add(5,6);
            timerStrip.add(6, 7);
            timerStrip.add(7,8);
            timerStrip.add(8, 9);
            timerStrip.add(9, 10);
            timerStrip.add(10, 11);
            timerStrip.add(11, 12);
            timerStrip.add(12, 13);
            timerStrip.add(13, 14);
            timerStrip.add(14, 15);
            timerStrip.add(15, 16);
            timerStrip.add(16, 17);
            timerStrip.add(17, 18);
            timerStrip.add(18, 19);
            timerStrip.add(19, 20);
            timerStrip.add(20, 21);
            timerStrip.add(21, 22);
            timerStrip.add(22, 23);
            timerStrip.add(23, 24);
            timerStrip.add(24, 25);
            timerStrip.add(25, 26);
            timerStrip.add(26, 27);
            timerStrip.add(27, 28);
            timerStrip.add(28, 29);
            timerStrip.add(29, 30);
        } else {
            centerPivotState = new SwerveState(17.6, 23.5);

            currentState = centerPivotState;

            swerveVelocityMapper.add(0.8, 0.027);
            swerveVelocityMapper.add(3.7, 0.035);
            swerveVelocityMapper.add(13.6, 0.067);
            swerveVelocityMapper.add(20.8, 0.1);
            swerveVelocityMapper.add(25.2, 0.119);
            swerveVelocityMapper.add(62.6, 0.3);
            swerveVelocityMapper.add(83.8, 0.399);
            swerveVelocityMapper.add(93.9, 0.45);
            swerveVelocityMapper.add(167.9, 0.8);
            swerveVelocityMapper.add(187.7, 0.9);
            swerveVelocityMapper.add(195.9, 0.94);

            // Swerves
            frontLeft = new SwerveModule(41, 51, 01, 0, rLog, RobotMotorType.TalonFX, RobotMotorType.TalonFX,
                    RobotEncoderType.Cancoder, 3.91, 6.12, 28.14);
            frontRight = new SwerveModule(42, 52, 2, 0, rLog, RobotMotorType.TalonFX, RobotMotorType.TalonFX,
                    RobotEncoderType.Cancoder, 3.91, 6.12, 28.14);
            backRight = new SwerveModule(43, 53, 3, 0, rLog, RobotMotorType.TalonFX, RobotMotorType.TalonFX,
                    RobotEncoderType.Cancoder, 3.91, 6.12, 28.14);
            backLeft = new SwerveModule(44, 54, 4, 0, rLog, RobotMotorType.TalonFX, RobotMotorType.TalonFX,
                    RobotEncoderType.Cancoder, 3.91, 6.12, 28.14);

            frontLeft.setVelocityMapper(swerveVelocityMapper);
            frontRight.setVelocityMapper(swerveVelocityMapper);
            backRight.setVelocityMapper(swerveVelocityMapper);
            backLeft.setVelocityMapper(swerveVelocityMapper);
            
            maxTeleopInchesPerSecond = 130.0;

            pdp = new PowerDistribution();
            pdp.setSwitchableChannel(true);
            distSensor = null; // new TimeOfFlight(0);
            yellowButton1 = new DigitalInput(3);
            yellowButton2 = new DigitalInput(4);
        }

        if (fullSwerve) {
            swerveDrives = new SwerveModule[] { frontLeft, frontRight, backRight, backLeft };
        } else {
            swerveDrives = new SwerveModule[] { backLeft };
        }

        commandTimer.start();
        if (distSensor != null)
            distSensor.setRangingMode(RangingMode.Short, 24);

        if (pdp != null) {
            batteryVoltage = pdp.getVoltage();
        }

        //This is so assigning new RGB segments can be done in other classes
        robotState.ledString = ledString;
        robotState.ledBuffer = ledBuffer;
        
        batteryTimer.start();
        ledString.setLength(ledLength);
        ledString.setData(ledBuffer);
        ledString.start();
        lidar = new LidarCamera("lidar");
    }

    public void stopSwerveMotors() {
        lastTravelVelocity = 0;
        lastTravelRotationDegreesPerSecond = 0;
        lastCommandTime = commandTimer.get();
        for (SwerveModule m : swerveDrives) {
            m.setDriveSpeed(0);
            m.setTurnPower(0);
            if (Math.abs(m.getDriveVelocity()) <= 0.25) {
                // If we are trying to stop the motor and the motor
                // is still not moving, then use Coast Mode
                m.setDriveAndTurnNeutralMode(false);
            } else {
                // If we are trying to stop the motor and the motor
                // is still spinning, then use Break Mode
                m.setDriveAndTurnNeutralMode(false);
            }
        }
    }

    public void alignSwerveMotorsForward() {
        lastTravelVelocity = 0;
        lastTravelAngleDegrees = 0;
        lastTravelRotationDegreesPerSecond = 0;
        lastCommandTime = commandTimer.get();
        for (SwerveModule m : swerveDrives) {
            m.setDrivePercent(0);
            m.setTurnHeading(0);
        }
    }

    public void stopAllMotors() { // Stops other motors, too
        stopSwerveMotors();
        // Stop all other motors too
    }

    private SwerveModule getModule(int module) {
        if (module >= swerveDrives.length || module < 0) {
            return null;
        } else {
            return swerveDrives[module];
        }
    }

    public void setDrivePercent(int moduleNumber, double power) {
        SwerveModule swerveDriveModule = getModule(moduleNumber);
        if (swerveDriveModule == null) {
            rLog.print("Not Valid Module ");
        } else {
            swerveDriveModule.setDrivePercent(power);
        }
    }

    public void setDriveSpeed(int moduleNumber, double velocity) {
        SwerveModule swerveDriveModule = getModule(moduleNumber);
        if (swerveDriveModule == null) {
            rLog.print("Not Valid Module ");
        } else {
            swerveDriveModule.setDriveSpeed(velocity);
        }
    }

    public RobotMotor getSwerveDriveMotor(int moduleNumber) {
        SwerveModule swerveDriveModule = getModule(moduleNumber);
        if (swerveDriveModule == null) {
            rLog.print("Not Valid Wheel");
            return null;
        } else {
            return swerveDriveModule.getDriveMotor();
        }
    }

    public void setTurnPercent(int moduleNumber, double power) {
        SwerveModule swerveDriveModule = getModule(moduleNumber);
        if (swerveDriveModule == null) {
            rLog.print("Not Valid Wheel");
        } else {
            swerveDriveModule.setTurnPower(power);
        }
    }

    public void setTurnHeading(int moduleNumber, double angle) {
        SwerveModule swerveDriveModule = getModule(moduleNumber);
        if (swerveDriveModule == null) {
            rLog.print("Not Valid Wheel");
        } else {
            swerveDriveModule.setTurnHeading(angle);
        }
    }

    public void setAllHeadings(double angle) {
        for (int i = 0; i < swerveDrives.length; i++) {
            setTurnHeading(i, angle);
        }
    }

    public void setTurnEncoder(int moduleNumber, double angle) {
        SwerveModule swerveDriveModule = getModule(moduleNumber);
        if (swerveDriveModule == null) {
            rLog.print("Not Valid Wheel");
        } else {
            swerveDriveModule.setTurnEncoderValue(angle);
        }
    }

    public void assignAllEncoderValues() {
        for (int i = 0; i < 4; i++) {
            setTurnEncoder(i, getAbsoluteTurnEncoderPosition(i));
        }
        rLog.print("Swerve Drive Turn Encoders Reset");
    }

    public void setAbsoluteEncoderValue(int moduleNumber, double angle) {
        SwerveModule swerveDriveModule = getModule(moduleNumber);
        if (swerveDriveModule == null) {
            rLog.print("Not Valid Wheel");
        } else {
            swerveDriveModule.setAbsoluteTurnEncoderPosition(angle);
        }
    }

    public void zeroAllAbsoluteEncoderValues() {
        for (int i = 0; i < 4; i++) {
            setAbsoluteEncoderValue(i, 0);
        }
    }

    public void setAllTurnEncoders(double angle) {
        for (int i = 0; i < swerveDrives.length; i++) {
            setTurnEncoder(i, angle);
        }
    }

    public double[] getAllTurnEncoders() {
        double[] positions = new double[swerveDrives.length];
        for (int i = 0; i < swerveDrives.length; i++) {
            positions[i] = swerveDrives[i].getTurnEncoderPosition();
        }
        return positions;
    }

    public double[] getAllDriveEncoders() {
        double[] positions = new double[swerveDrives.length];
        for (int i = 0; i < swerveDrives.length; i++) {
            positions[i] = swerveDrives[i].getDriveEncoderPosition();
        }
        return positions;
    }

    public double getDriveVelocity(int moduleNumber) {
        SwerveModule swerveDriveModule = getModule(moduleNumber);
        if (swerveDriveModule == null) {
            rLog.print("Not Valid Wheel");
            return 0;
        } else {
            return swerveDriveModule.getDriveVelocity();
        }
    }

    public double getDriveOutputPercentage(int moduleNumber) {
        SwerveModule swerveDriveModule = getModule(moduleNumber);
        if (swerveDriveModule == null) {
            rLog.print("Not Valid Wheel");
            return 0;
        } else {
            return swerveDriveModule.getDriveOutputPercentage();
        }
    }

    public boolean areWheelsStopped() {
        for (SwerveModule m : swerveDrives) {
            if (Math.abs(m.getDriveVelocity()) > 0.5) {
                return false;
            }
        }
        return true;
    }

    public double getAbsoluteTurnEncoderPosition(int moduleNumber) {
        SwerveModule swerveDriveModule = getModule(moduleNumber);
        if (swerveDriveModule == null) {
            rLog.print("Not Valid Wheel");
            return 0;
        } else {
            return swerveDriveModule.getAbsoluteTurnEncoderPosition();
        }
    }

    public double getTurnEncoderPosition(int moduleNumber) {
        SwerveModule swerveDriveModule = getModule(moduleNumber);
        if (swerveDriveModule == null) {
            rLog.print("Not Valid Wheel");
            return 0;
        } else {
            return swerveDriveModule.getTurnEncoderPosition();
        }
    }

    public void assignRobotMotionField(double travelAngle, double travelInchesPerSecond, double degreesPerSecond) {
        lastTravelVelocity = travelInchesPerSecond;
        lastTravelAngleDegrees = travelAngle;
        lastTravelRotationDegreesPerSecond = degreesPerSecond;
        lastCommandTime = commandTimer.get();
        currentState.assignSwerveModulesField(travelAngle, travelInchesPerSecond, degreesPerSecond, getGyro(),
                swerveDrives[0].getMaxVelocity());
        for (int i = 0; i < swerveDrives.length; i++) {
            swerveDrives[i].setSpeedAndHeading(currentState.getMagnitude(i), currentState.getAngle(i));
        }
    }

    public void assignRobotMotionRobot(double travelAngle, double travelInchesPerSecond, double degreesPerSecond) {
        lastTravelVelocity = travelInchesPerSecond;
        lastTravelAngleDegrees = travelAngle + getGyro();
        lastTravelRotationDegreesPerSecond = degreesPerSecond;
        lastCommandTime = commandTimer.get();
        currentState.assignSwerveModules(travelAngle, travelInchesPerSecond, degreesPerSecond,
                swerveDrives[0].getMaxVelocity());
        for (int i = 0; i < swerveDrives.length; i++) {
            swerveDrives[i].setSpeedAndHeading(currentState.getMagnitude(i), currentState.getAngle(i));
        }
    }

    public void assignRobotMotionAndHeadingField(double travelAngle, double travelInchesPerSecond, double facingAngle) {
        double degreesPerSecond = 0;
        double angleDifference = facingAngle - getGyroCenteredOnGoal(facingAngle);
        double acceptableRange = Math.max(Math.min((0.1042 * travelInchesPerSecond), 0.6), 0.6);
        if (Math.abs(angleDifference) < acceptableRange) {
            degreesPerSecond = 0;
        } else {
            double calcDPS = angleDifference * 4.0;
            double rotationSpeed = 180;
             if (calcDPS < 0) {
                degreesPerSecond = Math.min(Math.max(-rotationSpeed, calcDPS), -3.5);
            } else {
                degreesPerSecond = Math.max(Math.min(rotationSpeed, calcDPS), 3.5);
            }
        }
        assignRobotMotionField(travelAngle, travelInchesPerSecond, degreesPerSecond);
    }

    public void assignRobotMotionAndHeadingFieldAccel(double travelAngle, double travelInchesPerSecond,
            double facingAngle, double maxTravelAcceleration) {
        double degreesPerSecond = 0;
        double angleDifference = facingAngle - getGyroCenteredOnGoal(facingAngle);
        double acceptableRange = Math.max(Math.min((0.1042 * travelInchesPerSecond), 2.5), 1);
        if (Math.abs(angleDifference) < acceptableRange) {
            degreesPerSecond = 0;
        } else {
            double calcDPS = angleDifference * 2.0;
            if (calcDPS < 0) {
                degreesPerSecond = Math.min(Math.max(-120, calcDPS), -25);
            } else {
                degreesPerSecond = Math.max(Math.min(120, calcDPS), 25);
            }
        }
        assignRobotMotionField(travelAngle, limitAccel(travelInchesPerSecond, maxTravelAcceleration), degreesPerSecond);
    }

    public void lockWheels() {
        lastTravelVelocity = 0;
        lastTravelRotationDegreesPerSecond = 0;
        lastCommandTime = commandTimer.get();
        currentState.lockWheels();
        for (int i = 0; i < swerveDrives.length; i++) {
            swerveDrives[i].setDrivePercent(currentState.getMagnitude(i));
            swerveDrives[i].setTurnHeading(currentState.getAngle(i));
        }
    }

    // swerveModuleNumber: 0 - frontleft; 1 - frontright; 2 - backright; 3 -
    // backleft
    // Type: 0 - Not Applicable; 1 - Turn; 2 - Drive
    // OtherMotorID: 0 - Not Applicable; Use motor IDs
    public boolean isMotorAttached(boolean isSwerve, int swerveModuleNumber, int swerveType, int notSwerveMotorID) {
        if (isSwerve == true) {
            SwerveModule swerveDriveModule = getModule(swerveModuleNumber);
            if (swerveDriveModule == null) {
                rLog.print("Not Valid Swerve Module Selected, Module: " + swerveModuleNumber);
                return false;
            } else {
                if (swerveType == 1)
                    return swerveDriveModule.isTurnAttached();
                else if (swerveType == 2)
                    return swerveDriveModule.isDriveAttached();
                else
                    return false;
            }
        } else if (isSwerve == false) {
            return false;
        } else {
            return false;
        }
    }

    public double getLastTravelVelocityCommand() {
        return lastTravelVelocity;
    }

    public double getLastTravelAngleDegrees() {
        return lastTravelAngleDegrees;
    }

    public double getLastTravelRotationDegreesPerSecond() {
        return lastTravelRotationDegreesPerSecond;
    }

    public double getCalculatedTravelSinceLastCommand() {
        return lastTravelVelocity * (commandTimer.get() - lastCommandTime);
    }

    public double getMaxSpeedChange(double maxTravelAcceleration) {
        return maxTravelAcceleration * (commandTimer.get() - lastCommandTime);
    }

    public double limitAccel(double goalInPerSec, double maxTravelAcceleration) {
        double calcInPerSec;
        if (goalInPerSec >= getLastTravelVelocityCommand()) {
            calcInPerSec = Math.min(goalInPerSec,
                    getLastTravelVelocityCommand() + getMaxSpeedChange(maxTravelAcceleration));
        } else {
            calcInPerSec = Math.max(goalInPerSec,
                    getLastTravelVelocityCommand() - getMaxSpeedChange(maxTravelAcceleration));
        }
        return calcInPerSec;
    }

    public double getMaxTeleopInchesPerSecond() {
        return maxTeleopInchesPerSecond;
    }

    public void setSwerveCenterPoint(double x, double y) {
        currentState = new SwerveState(centerPivotState, x, y);
    }

    public void setSwerveCoast() {
        for (SwerveModule m : swerveDrives) {
            m.setDriveAndTurnNeutralMode(true);
        }
    }

    public void setSwerveBrake() {
        for (SwerveModule m : swerveDrives) {
            m.setDriveAndTurnNeutralMode(false);
        }
    }

    public double getGyro() {
        if (gyro != null)
            return gyro.getAngle();
        else
            return 0.0;
    }

    public double getGyroCenteredOnGoal(double goalAngle) {
        double gyroValue = getGyro();

        return RobotMath.angleCenteredOnTarget(gyroValue, goalAngle);
    }

    public void setGyro(double gyroAngle) {
        if (gyro != null)
            gyro.setYaw(gyroAngle);
    }

    public void resetGyro() {
        setGyro(0);
    }

    public boolean isBatteryGood() {
        if (batteryVoltage >= 12.4) {
            return true;
        } else {
            return false;
        }
    }

    public boolean isDistanceSensorRangeValid() {
        if (distSensor != null)
            return distSensor.isRangeValid();
        else
            return false;
    }

    public double getDistanceInInches() {
        double distance;
        if (distSensor != null) {
            distance = distSensor.getRange();
        } else {
            distance = 0;
        }
        distance = distance * 0.0393701; // Convert mm to inches
        double robotCenterDistance = distance + DISTANCE_SENSOR_OFFSET;
        return robotCenterDistance;
    }

    public boolean getYellowButtonValue() {
        if (yellowButton1 == null || yellowButton2 == null) {
            return false;
        }
        boolean bothPressed = yellowButton1.get() && yellowButton2.get();
        boolean yellowButtonValue = bothPressed && !lastYellowButtonsValue;

        lastYellowButtonsValue = bothPressed;

        return yellowButtonValue;
    }

    public String getRobotName() {
        String name;
        if (robotName == RobotName.JerryJr) {
            name = "Jerry Jr";
        } else if (robotName == RobotName.Scorpio) {
            name = "Invisibot";
        } else {
            name = "Live 2024";
        }
        return name;
    }

    public boolean isRedAlliance() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        return (alliance.isPresent() && alliance.get() == Alliance.Red);
    }

    public void robotPeriodic() {
        if (batteryTimer.get() >= 10 && pdp != null) {
            batteryVoltage = pdp.getVoltage();
            batteryTimer.reset();
            batteryTimer.start();
        }
    }

    public void setLED(LEDregion region, LEDcolor color) {
        int start = 0;
        int end = 0;
        if (region == LEDregion.regionOne) {
            start = 10;
            end = 91;
        } else if (region == LEDregion.regionTwo) {
            start = 160;
            end = 240;
        } else {
            start = 0;
            end = 0;
        }

        if (end >= ledLength)
            end = ledLength - 1;
        if (start <= 0)
            start = 0;
        if (color == LEDcolor.BadRed) {
            if (LEDStrobe == 1) {
                rgb = new Color(255, 0, 0);
                LEDStrobe = LEDStrobe * -1;
            } else if (LEDStrobe == -1) {
                rgb = new Color(0, 0, 0);
                LEDStrobe = LEDStrobe * -1;
            }
        }
        if (color == LEDcolor.Red) {
            rgb = new Color(150, 0, 0);
        } else if (color == LEDcolor.DimRed) {
            rgb = new Color(20, 0, 0);
        } else if (color == LEDcolor.Blue) {
            rgb = new Color(0, 0, 255);
        } else if (color == LEDcolor.DimBlue) {
            rgb = new Color(0, 0, 20);
        } else if (color == LEDcolor.Green) {
            rgb = new Color(0, 150, 0);
        } else if (color == LEDcolor.GreenBlue) {
            rgb = new Color(0, 150, 70);
        } else if (color == LEDcolor.Yellow) {
            rgb = new Color(120, 120, 0);
        } else if (color == LEDcolor.Black) {
            rgb = new Color(0, 0, 0);
        } else if (color == LEDcolor.Purple) {
            rgb = new Color(91, 0, 127);
        } else if (color == LEDcolor.White) {
            rgb = new Color(100, 100, 100);
        } else if (color == LEDcolor.DimWhite) {
            rgb = new Color(15, 15, 15);
        } else if (color == LEDcolor.Orange) {
            rgb = new Color(255, 80, 0);
        }
        for (int i = start; i <= end; i++) {
            ledBuffer.setLED(i, rgb);
        }
    }

    /**
     * This method will assign a new LED string and make an RGB section on it.
     * To use this method, you need to make an RGB variable equal to it, then use the variable.
     * 
     * <p>Note: Importing Electronics.RGB is required to use it in other files
     * 
     * @param port Which port to use in the addressableLED constructor
     * @param bufferLength How long to make the new buffer
     * @param firstLED The first LED in the segment you want to be RGB
     * @param lastLED The last LED in the segment you want to be RGB
     * @param ledLength The length of the ENTIRE LED string you're using.
     * @param changeSpeed How quickly RGB cycles through colors (while the technical limit is 32727, anything
     *   past 255 is simply garish)
     * @return Adjusted variables for what the new RGB will use
     */
    public RGB assignNewRGBString(int port, int bufferLength, int firstLED, int lastLED, int ledLength, short changeSpeed) {
        RGB rgb = new RGB();
        rgb.rgbString = new AddressableLED(port);
        rgb.rgbBuffer = new AddressableLEDBuffer(bufferLength);
        rgb.startingLED = firstLED;
        rgb.endingLED = lastLED;
        rgb.ledLength = ledLength;
        if (changeSpeed > 0)
            rgb.changeSpeed = changeSpeed;
        else
            rgb.changeSpeed = 1;
        rgb.rgbString.setLength(ledLength);
        rgb.rgbString.setData(rgb.rgbBuffer);
        rgb.rgbString.start();
        return rgb;
    }

    /**
     * This method will assign an RGB segment to an existing RGB string.
     * To use this method, you need to make an RGB variable equal to it, then use the variable.
     * 
     * <p>Note: For now, I made copies of the LEDString assigned above to robotState so that other classes have access to them.
     * 
     * <p>Note: Importing Electronics.RGB is required to use it in other files
     * 
     * @param addressableLED Which addressable LED to use
     * @param buffer Which LED buffer to use
     * @param firstLED The first LED in the segment you want to be RGB
     * @param lastLED The last LED in the segment you want to be RGB
     * @param changeSpeed How quickly RGB cycles through colors (while the technical limit is 32727, anything
     *   past 255 is simply garish)
     * @return Adjusted variables for what the new RGB will use
     */
    public RGB assignExistingRGBString(AddressableLED addressableLED, AddressableLEDBuffer buffer, 
            int firstLED, int lastLED, short changeSpeed) {
        RGB rgb = new RGB();
        rgb.rgbString = addressableLED;
        rgb.rgbBuffer = buffer;
        rgb.startingLED = firstLED;
        rgb.endingLED = lastLED;
        rgb.ledLength = ledLength;
        if (changeSpeed > 0)
            rgb.changeSpeed = changeSpeed;
        else
            rgb.changeSpeed = 1;
        return rgb;
    }

    public void setLEDsForCountdown(double timeRemaining) {
        Color color1 = new Color(10, 20, 30);
        Color color2 = new Color(30, 10, 10);
        for (SingleLED led : timerStrip.LEDcount) {
            //rLog.print("LED:" + led.number + "," + led.time + "," + timeRemaining);
            if (timeRemaining < led.time) {
                ledBuffer.setLED(led.number, color1);
            } else {
                ledBuffer.setLED(led.number, color2);
            }
        }
    }

    public void sendLEDBuffer() {
        ledString.setData(ledBuffer);
    }

    /**
     * Note to self: If the gyro's ever put back the right way, switch pitch and roll again
     * -Will
     */
    public double getRoboPitch() {
        if (gyro != null)
            return gyro.getRoll() - rollOffset;
        else
            return 0;
    }

    /**
     * Note to self: If the gyro's ever put back the right way, switch pitch and roll again
     * -Will
     */
    public double getRoboRoll() {
        if (gyro != null)
            return gyro.getPitch() - pitchOffset;
        else
            return 0;
    }

    public void resetTilt() {
        if (gyro == null)
            return;
        else {
            pitchOffset = gyro.getPitch();
            rollOffset = gyro.getRoll();
        }
    }

    public boolean isLidarConnected() {
        if (lidar != null)
            return lidar.isConnected();
        else
            return false;
    }
}