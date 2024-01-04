// TeleopRobot.java - Code for teleop mode
package frc.robot;

import edu.wpi.first.wpilibj.*;
import frc.robot.RobotEnums.*;
import frc.robot.Electronics.RGB;

public class TeleopRobot {
    private Electronics e;
    private RoboLog rLog;
    private Action act;
    private OI oi;
    private RobotState robotState;
    private Dashboard dash;

    // Trying to get dashboard to control a certain aspect of these
    public RGB leftRGB;
    public RGB rightRGB;

    private double facing = 0;
    private boolean isFacing0 = false;
    private boolean isFacing180 = false;
    private boolean lastAlignWheels = false;
    private boolean lastLockWheels = false;
    private boolean lastDriverResetGyro = false;
    private boolean lastDriverReverseResetGyro = false;
    private boolean lastDriveFast = false;
    private boolean lastDriveSlow = false;
    private boolean lastDriveVerySlow = false;

    private enum TeleopProgram {
        None,
        TeleopOne
    }

    private TeleopProgram whichAutoProgram;
    private int teleopStepNumber;
    private boolean isAutoTeleopRunning = false;

    private double lastDriveSpeed = 0.0;
    private double lastDriveDirection = 0.0;

    public TeleopRobot(Electronics e, RoboLog rLog, Action act, OI oi,
            RobotState robotState, Dashboard dash) {
        this.e = e;
        this.rLog = rLog;
        this.act = act;
        this.oi = oi;
        this.robotState = robotState;
        this.dash = dash;

        if (this.rLog == null) {
            System.out.println("Warning: TeleopRobot.rLog is null");
        }
        if (this.e == null) {
            rLog.print("Warning: TeleopRobot.e is null");
        }
        if (this.act == null) {
            rLog.print("Warning: TeleopRobot.act is null");
        }
        if (this.oi == null) {
            rLog.print("Warning: TeleopRobot.oi is null");
        }
        if (this.robotState == null) {
            rLog.print("Warning: TeleopRobot.robotState is null");
        }
        if (this.dash == null) {
            rLog.print("Warning: TeleopRobot.dash is null");
        }
    }

    public void teleopInit() {
        robotState.startLoggingAprilTags();
        act.actionReset();
        lastDriveSpeed = 0.0;
        lastDriveDirection = 0.0;
        teleopStepNumber = 1;
        isAutoTeleopRunning = false;
        isFacing0 = false;
        isFacing180 = false;
        leftRGB = e.assignExistingRGBString(robotState.ledString, robotState.ledBuffer, 10, 91, (short) 1);
        rightRGB = e.assignExistingRGBString(robotState.ledString, robotState.ledBuffer, 160, 240, (short) 1);
    }

    private void logButtonChanges() {
        boolean alignWheels = oi.getAlignWheelsButtonPressed();
        boolean lockWheels = oi.getLockButtonPressed();
        boolean driverResetGyro = oi.getDriverResetGyroButtonPressed();
        boolean driverReverseResetGyro = oi.getDriverReverseResetGyroButtonPressed();
        boolean driveFast = oi.getDriveFastButtonPressed();
        boolean driveSlow = oi.getDriveSlowButtonPressed();
        boolean driveVerySlow = oi.getDriveVerySlowButtonPressed();

        if (alignWheels != lastAlignWheels) {
            if (alignWheels)
                rLog.print("AlignWheelsButtonPressed");
            else
                rLog.print("AlignWheelsButtonReleased");
        }

        if (lockWheels != lastLockWheels) {
            if (lockWheels)
                rLog.print("LockWheelsButtonPressed");
            else
                rLog.print("LockWheelsButtonReleased");
        }

        if (driverResetGyro != lastDriverResetGyro) {
            if (driverResetGyro)
                rLog.print("DriverResetGyroButtonPressed");
            else
                rLog.print("DriverResetGyroButtonReleased");
        }

        if (driverReverseResetGyro != lastDriverReverseResetGyro) {
            if (driverReverseResetGyro)
                rLog.print("DriverReverseResetGyroButtonPressed");
            else
                rLog.print("DriverReverseResetGyroButtonReleased");
        }

        if (driveFast != lastDriveFast) {
            if (driveFast)
                rLog.print("driveFast Pressed");
            else
                rLog.print("driveFast Released");
        }
        if (driveSlow != lastDriveSlow) {
            if (driveSlow)
                rLog.print("driveSlow Pressed");
            else
                rLog.print("driveSlow Released");
        }
        if (driveVerySlow != lastDriveVerySlow) {
            if (driveVerySlow)
                rLog.print("driveVerySlow Pressed");
            else
                rLog.print("driveVerySlow Released");
        }

        lastAlignWheels = alignWheels;
        lastLockWheels = lockWheels;
        lastDriverResetGyro = driverResetGyro;
        lastDriverReverseResetGyro = driverReverseResetGyro;
        lastDriveFast = driveFast;
        lastDriveSlow = driveSlow;
        lastDriveVerySlow = driveVerySlow;
    }

    void setTeleopStepNumber(int number) {
        teleopStepNumber = number;
        act.actionReset();
        rLog.print("New Teleop Step: " + teleopStepNumber + ", Program: " + whichAutoProgram
                + ", Gyro: " + RobotMath.round2(e.getGyro()));
    }

    public void teleopPeriodic() {
        logButtonChanges();
        double joyMag = Math.pow(oi.getDriveMagnitude(), 2);
        double driveSpeed = 0.0;
        double driveDirection = lastDriveDirection;
        boolean isStopped = false;
        LEDcolor leftColor = LEDcolor.Black;
        LEDcolor rightColor = LEDcolor.Black;

        leftRGB.rgbToggle(oi.getRGBButtonPressed());
        rightRGB.rgbToggle(oi.getRGBButtonPressed());

        leftColor = LEDcolor.Black;
        rightColor = LEDcolor.Black;

        if (isAutoTeleopRunning) {
            switch (whichAutoProgram) {
                case None: // Aborts autoTeleop
                    isAutoTeleopRunning = false;
                    break;

                case TeleopOne:
                    switch (teleopStepNumber) {
                        case 1: // TeleopOne: Step 1
                            if (act.driveStraightWithFacing(0, 10, 0,
                                    100, 12, 0))
                                setTeleopStepNumber(2);
                            break;
                        case 2: // TeleopOne: Done
                            isAutoTeleopRunning = false;
                            rLog.print(whichAutoProgram + " completed");
                            break;
                    }
                    break;
            }
        } else {
            whichAutoProgram = TeleopProgram.None;
        }

        // Teleop driving, driver can't control robot in autoTeleop mode
        if (isAutoTeleopRunning == false) {
            if (oi.getDriverResetGyroButtonPressed() || oi.getDriverReverseResetGyroButtonPressed()) {
                isFacing0 = false;
                isFacing180 = false;
                facing = e.getGyro();
            }

            if (oi.getDriveFastButtonPressed() || oi.getDriveSlowButtonPressed()
                    || oi.getDriveVerySlowButtonPressed()) { // 6 & axis 3 & 4
                if (joyMag < 0.2) {
                    driveSpeed = 0;
                } else {
                    double maxSpeed = (oi.getDriveVerySlowButtonPressed() ? 30
                            : oi.getDriveSlowButtonPressed() ? 60 : e.getMaxTeleopInchesPerSecond());
                    driveSpeed = (joyMag - 0.2) * (maxSpeed / 0.8);
                    if (isFacing0) {
                        facing = 0;
                    } else if (isFacing180) {
                        facing = 180;
                    }
                }
                driveDirection = oi.getDriveDirectionDegrees();
                double facingMagnitude = oi.getFacingJoystickMagnitude();
                double facingDegrees = oi.getFacingJoystickDegrees();
                rLog.print("Facing: " + facingMagnitude + " , " + facingDegrees);

                if (facingMagnitude > 0.6) {
                    if (facingDegrees < 45.0 || facingDegrees > 315.0) {
                        facing = 0;
                        isFacing0 = true;
                        isFacing180 = false;
                    } else if (facingDegrees < 135.0) {
                        facing = 90;
                        isFacing0 = false;
                        isFacing180 = false;
                    } else if (facingDegrees < 225.0) {
                        facing = 180;
                        isFacing0 = false;
                        isFacing180 = true;
                    } else if (facingDegrees < 315.0) {
                        facing = 270;
                        isFacing0 = false;
                        isFacing180 = false;
                    } else {
                        isFacing0 = false;
                        isFacing180 = false;
                    }
                }

                isStopped = false;
            } else {
                facing = e.getGyro();
                driveSpeed = 0.0;
                driveDirection = lastDriveDirection;
                isStopped = true;
            }

            // Ramp down to prevent tipping robot
            if (lastDriveSpeed > 50) {
                double angleDiff = Math.abs(RobotMath.angleCenteredOnTarget(driveDirection, lastDriveDirection) -
                        lastDriveDirection);
                if ((driveSpeed < lastDriveSpeed - 7.5) || (angleDiff > 35.0)) {
                    driveSpeed = lastDriveSpeed - 7.5;
                    driveDirection = lastDriveDirection;
                    isStopped = false;
                }
            }

            lastDriveSpeed = driveSpeed;
            lastDriveDirection = driveDirection;

            if (!isStopped) {
                e.assignRobotMotionAndHeadingField(driveDirection, driveSpeed, facing);
            } else if (oi.getAlignWheelsButtonPressed()) {
                e.alignSwerveMotorsForward();
            } else if (oi.getLockButtonPressed()) {
                e.lockWheels();
            } else {
                e.stopSwerveMotors();
            }
        }

        if (rightColor == LEDcolor.Black) {
            rightRGB.smoothRGBActivate();
        } else {
            e.setLED(LEDregion.regionTwo, rightColor);
        }
        if (leftColor == LEDcolor.Black) {
            leftRGB.smoothRGBActivate();
        } else {
            e.setLED(LEDregion.regionOne, leftColor);
        }

        // Display the countdown using the LEDs
        double timeRemaining = DriverStation.getMatchTime();
        if (timeRemaining > 0 && timeRemaining < 30.0) {
            e.setLEDsForCountdown(DriverStation.getMatchTime());
        }
    }
}