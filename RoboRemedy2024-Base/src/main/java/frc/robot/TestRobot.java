// TestRobot.java - Code for test mode
package frc.robot;

import edu.wpi.first.wpilibj.*;
import frc.robot.RobotEnums.*;

public class TestRobot {
    private Electronics e;
    private RoboLog rLog;
    private Action act;
    private Timer timer1 = new Timer();
    private Dashboard d;
    private RobotState robotState;
    private OI oi;
    private boolean isButtonPressed = false;
    private boolean lastButtonPressed = false;

    public TestRobot(Electronics e, RoboLog rLog, Action act, Dashboard d, RobotState robotState, OI oi) {
        this.e = e;
        this.rLog = rLog;
        this.act = act;
        this.d = d;
        this.oi = oi;
        this.robotState = robotState;
        
        if (this.rLog == null) {
            System.out.println("Warning: TestRobot.rLog is null"); 
        }
        if (this.e == null) {
            rLog.print("Warning: TestRobot.e is null");
        }
        if (this.act == null) {
            rLog.print("Warning: TestRobot.act is null");
        }
        if (this.d == null) {
            rLog.print("Warning: TestRobot.d is null");
        }
        if (this.oi == null) {
            rLog.print("Warning: TestRobot.oi is null");
        }
        if (this.robotState == null) {
            rLog.print("Warning: TestRobot.robotState is null");
        }
    }

    public void testInit() {
        act.actionReset();
        timer1.reset();
        timer1.start();
        rLog.print("Start Test Program: " + d.getTestMode());
        e.stopAllMotors();
        lastButtonPressed = false;
    }

    public void testPeriodic() {
        if (d.getTestMode() == TestMode.QuickTest)
            quickTest();   
        else if (d.getTestMode() == TestMode.SwerveTest)
            testSwerve();
        else if (d.getTestMode() == TestMode.SwerveDrive)
            testSwerveDrive();
        else if (d.getTestMode() == TestMode.TestAngle)
            testAngle();
        else if (d.getTestMode() == TestMode.TestMotors)
            testMotors();
        else if (d.getTestMode() == TestMode.TuneMotor)
            testTuneMotor();
        else if (d.getTestMode() == TestMode.TestLED)
            LEDtest();
        else if (d.getTestMode() == TestMode.ResetEncoders)
            ResetEncoder();
        else if (d.getTestMode() == TestMode.TestCountdown)
            e.setLEDsForCountdown(30 - timer1.get());
    }

    //Add testing subjects in here to make a quick run
    public void quickTest() {
        d.setTestString("Enter text here");
    }

    public void testSwerve() {
        int module = 0;
        if (oi.isJoystickButtonPressed(0, 9)) {
            module = 1;
        } else if (oi.isJoystickPovPressed(0, 180)) {
            module = 2;
        } else if (oi.isJoystickPovPressed(0, 225)) {
            module = 3;
        }

        double pow = -1 * oi.getJoystickAxis(0, 1);
        double mag = oi.getMagnitude(0, 0, 1);
        double goalAng = oi.getDirectionDegrees(0, 0, 1);
        if (oi.isJoystickButtonPressed(0, 1)) {
            e.setDrivePercent(module, 0.2);
            e.setTurnPercent(module, 0);
        } else if (oi.isJoystickButtonPressed(0, 2)) {
            e.setTurnPercent(module, 0.2);
            e.setDrivePercent(module, 0);
        } else if (oi.isJoystickButtonPressed(0, 3)) {
            e.setDrivePercent(module, 0.8 * pow);
            e.setTurnPercent(module, 0);
        } else if (oi.isJoystickButtonPressed(0, 4)) {
            e.setTurnPercent(module, 0.8 * pow);
            e.setDrivePercent(module, 0);
        } else if (oi.isJoystickButtonPressed(0, 5)) {
            e.setAllHeadings(0);
        } else if (oi.isJoystickButtonPressed(0, 6)) {
            e.setAllHeadings(90);
        } else if (oi.isJoystickButtonPressed(0, 7)) {
            if (mag > 0.3) {
                e.setAllHeadings(goalAng);
            }
        } else if (oi.isJoystickPovPressed(0, 0)) {
            e.setDriveSpeed(module, 4);
            rLog.print("Velocity, cmd = 4 act = " + e.getDriveVelocity(module) +
                " out% = " + e.getDriveOutputPercentage(module));
        } else if (oi.isJoystickPovPressed(0, 45)) {
            e.setDriveSpeed(module, 12);
            rLog.print("Velocity, cmd = 12 act = " + e.getDriveVelocity(module) +
                " out% = " + e.getDriveOutputPercentage(module));
        } else if (oi.isJoystickPovPressed(0, 90)) {
            double velocity = pow * 100.0;
            e.setDriveSpeed(module, velocity);
            rLog.print("Velocity, cmd = " + velocity + " act = " + e.getDriveVelocity(module) +
                " out% = " + e.getDriveOutputPercentage(module));
        } else {
            e.stopSwerveMotors();
        }
        if (oi.isJoystickPovPressed(0, 270)) {
            rLog.print("Abs Encoders Front Left: " + e.getAbsoluteTurnEncoderPosition(0) + "Front Right: "
                    + e.getAbsoluteTurnEncoderPosition(1) + "Back Right: " + e.getAbsoluteTurnEncoderPosition(2)
                    + "Back Left: " + e.getAbsoluteTurnEncoderPosition(3));
        } else if (oi.isJoystickPovPressed(0, 315)) {
            rLog.print("non-ABS Encoedrs Front Left:" + e.getTurnEncoderPosition(0) + "Front Right: "
                    + e.getTurnEncoderPosition(1) + "Back Right: " + e.getTurnEncoderPosition(2)
                    + "Back Left: " + e.getTurnEncoderPosition(3));
        }
    }

    public void testSwerveDrive() {
        boolean isButtonPressed = false;
        if (oi.isJoystickButtonPressed(0, 1)) {
            e.assignRobotMotionRobot(0, 1.5, 0);
            isButtonPressed = true;
        } else if (oi.isJoystickButtonPressed(0, 2)) {
            e.assignRobotMotionRobot(0, 3.7, 0);
            isButtonPressed = true;
        } else {
            e.stopSwerveMotors();
        }

        if (isButtonPressed) {
            RobotMotor motor1 = e.getSwerveDriveMotor(1);
            RobotMotor motor3 = e.getSwerveDriveMotor(3);
            rLog.print("Speed1 = " + RobotMath.round2(motor1.getEncoderVelocity()) +
                    "Speed3 = " + RobotMath.round2(motor3.getEncoderVelocity()));
        }
    }

    public void testAngle() {
        if (oi.isJoystickButtonPressed(0, 1))
            e.assignRobotMotionAndHeadingField(0, 0, -1);
        else if (oi.isJoystickButtonPressed(0, 2))
            e.assignRobotMotionAndHeadingField(0, 0, -1.5);
        else if (oi.isJoystickButtonPressed(0, 3))
            e.assignRobotMotionAndHeadingField(0, 0, -3);
        else if (oi.isJoystickButtonPressed(0, 4))
            e.assignRobotMotionAndHeadingField(0, 0, -12);
        else
            e.stopSwerveMotors();
    }

    public void testMotors() {
        if (oi.isJoystickButtonPressed(0, 1)) {
            // Add first test here
        } else if (oi.isJoystickButtonPressed(0, 2)) {
            // Add second test here
        }
    }

    public void testTuneMotor() {
        double joystickValue = -oi.getJoystickAxis(0, 1);
        int joystickPOV = oi.getPOV(0);
        RobotMotor motor = e.getSwerveDriveMotor(1);
        double maxVelocity = motor.getPercentVelocityLinearMapper().getMaxInputValue();
        double maxPercentPerSecond = 0.5;
        double maxVelocityPerSecond = maxVelocity / 2;

        if (isButtonPressed) {
            double timeValue = RobotMath.round2(timer1.get());
            double lastPercent = motor.getLastAssignedPercentage();
            double lastVelocity = motor.getLastAssignedVelocity();
            double actualVelocity = RobotMath.round2(motor.getEncoderVelocity());
            if (lastPercent != 0.0) {
                rLog.print("Time: " + timeValue +
                        " Percent: " + lastPercent +
                        " Actual Velocity:" + actualVelocity);
            } else if (lastVelocity != 0.0) {
                double velocityError = RobotMath.round2((motor.getEncoderVelocity() -
                        motor.getLastAssignedVelocity()));
                rLog.print("Time: " + timeValue +
                        " Velocity: " + lastVelocity +
                        " Actual Velocity:" + actualVelocity +
                        " Err: " + velocityError);
            }
        }
        
        isButtonPressed = true;

        if (oi.isJoystickButtonPressed(0, 1)) {
            // Button 1 pressed - percentage output based on joystick
            motor.setPercent(joystickValue);
        } else if (oi.isJoystickButtonPressed(0, 2)) {
            // Button 2, assign percentage output based on POV
            switch (joystickPOV) {
                case 0:
                    // Button 2, POV=0 = percent output to 9%
                    motor.rampToPercent(0.03, maxPercentPerSecond);
                    break;
                case 45:
                    // Button 2, POV=45 = percent output to 12%
                    motor.rampToPercent(0.1, maxPercentPerSecond);
                    break;
                case 90:
                    // Button 2, POV=90
                    motor.rampToPercent(0.3, maxPercentPerSecond);
                    break;
                case 135:
                    // Button 2, POV=135
                    motor.rampToPercent(0.4, maxPercentPerSecond);
                    break;
                case 180:
                    // Button 2, POV=180
                    motor.rampToPercent(0.45, maxPercentPerSecond);
                    break;
                case 225:
                    // Button 2, POV=225 = percent output to 50%
                    motor.rampToPercent(0.8, maxPercentPerSecond);
                    break;
                case 270:
                    // Button 2, POV=270 = percent output to 75%
                    motor.rampToPercent(0.9, maxPercentPerSecond);
                    break;
                case 315:
                    // Button 2, POV=315 = percent output to 100%
                    motor.rampToPercent(0.94, maxPercentPerSecond);
                    break;
                default:
                    // No POV button pressed - turn off motors
                    isButtonPressed = false;
                    motor.rampToPercent(0.0, maxPercentPerSecond);
            }
        } else if (oi.isJoystickButtonPressed(0, 3)) {
            // Button 3, assign velocity based on POV
            switch (joystickPOV) {
                case 0:
                    // Button 3, POV=0 = velocity to minimum speed
                    motor.rampToVelocity(0.04 * maxVelocity, maxVelocityPerSecond);
                    break;
                case 45:
                    // Button 3, POV=45 = velocity to second lowest speed
                    motor.rampToVelocity(0.07 * maxVelocity, maxVelocityPerSecond);
                    break;
                case 90:
                    // Button 3, POV=90 = velocity to third lowest speed
                    motor.rampToVelocity(0.12 * maxVelocity, maxVelocityPerSecond);
                    break;
                case 135:
                    // Button 3, POV=135 = velocity to fourth lowest speed
                    motor.rampToVelocity(0.20 * maxVelocity, maxVelocityPerSecond);
                    break;
                case 180:
                    // Button 3, POV=180 = velocity to fifth lowest speed
                    motor.rampToVelocity(0.45 * maxVelocity, maxVelocityPerSecond);
                    break;
                case 225:
                    // Button 3, POV=225 = velocity to sixth lowest speed
                    motor.rampToVelocity(0.60 * maxVelocity, maxVelocityPerSecond);
                    break;
                case 270:
                    // Button 3, POV=270 = velocity to 75%
                    motor.rampToVelocity(0.75 * maxVelocity, maxVelocityPerSecond);
                    break;
                case 315:
                    // Button 3, POV=315 = velocity to 100%
                    motor.rampToVelocity(1.0 * maxVelocity, maxVelocityPerSecond);
                    break;
                default:
                    // No POV button pressed - turn off motors
                    isButtonPressed = false;
                    motor.rampToVelocity(0.0, maxVelocityPerSecond);
            }
        } else if (oi.isJoystickButtonPressed(0, 4)) {
            // Button 4 pressed - velocity control based on joystick
            motor.setVelocity(joystickValue * maxVelocity);
        } else {
            // No buttons pressed - turn off motors
            isButtonPressed = false;
            motor.stopMotor();
            motor.setIntegralAccumulator(0);
        }
    }

    public void LEDtest() {
        if (oi.isJoystickButtonPressed(0, 1)) {
            e.setLED(LEDregion.regionOne, LEDcolor.Yellow);
        } else if (oi.isJoystickButtonPressed(0, 2)) {
            e.setLED(LEDregion.regionOne, LEDcolor.Purple);
        } else if (oi.isJoystickButtonPressed(0, 3)) {
            e.setLED(LEDregion.regionOne, LEDcolor.Red);
        } else if (oi.isJoystickButtonPressed(0, 4)) {
            e.setLED(LEDregion.regionOne, LEDcolor.Black);
        } else if (oi.isJoystickButtonPressed(0, 5)) {
            e.setLED(LEDregion.regionOne, LEDcolor.Blue);
        }  
    }
    public void ResetEncoder () {
        boolean isPressed = oi.isJoystickButtonPressed(0, 1);
        if (isPressed && !lastButtonPressed) {
            e.zeroAllAbsoluteEncoderValues(); //reset swerve encoders
            e.assignAllEncoderValues();
            rLog.print("Reset Swerve Encoders pressed");
        }
        lastButtonPressed = isPressed;
    }
}  