// OI.java - Operator Interface. Reads values from joysticks and buttons
package frc.robot;

import edu.wpi.first.wpilibj.*;

public class OI {

    OI() {
    }
    
    public boolean isJoystickButtonPressed(int stick, int button) {
        if (DriverStation.isJoystickConnected(stick)) {
            return DriverStation.getStickButton(stick, button);
        } else {
            return false;
        }
    }

    public boolean isJoystickPovPressed(int stick, int value) {
        if (DriverStation.isJoystickConnected(stick)) {
            return (DriverStation.getStickPOV(stick, 0) == value);
        } else {
            return false;
        }
    }

    public boolean isJoystickTriggerPressed(int stick, int value) {
        if (DriverStation.isJoystickConnected(stick)) {
            if (DriverStation.getStickAxis(stick, value) > 0.5) {
                return true;
            } else {
                return false;
            }
        } else {
            return false;
        }
    }

    public boolean isJoystickTriggerNegativePressed(int stick, int value) {
        if (DriverStation.isJoystickConnected(stick)) {
            if (DriverStation.getStickAxis(stick, value) < -0.5) {
                return true;
            } else {
                return false;
            }
        } else {
            return false;
        }
    }

    public double getJoystickAxis(int stick, int axis) {
        if (DriverStation.isJoystickConnected(stick)) {
            return DriverStation.getStickAxis(stick, axis);
        } else {
            return 0;
        }
    }

    public int getPOV(int stick) {
        if (DriverStation.isJoystickConnected(stick)) {
            return (DriverStation.getStickPOV(stick, 0));
        } else {
            return 0; 
        }
    }

    double getMagnitude(int stick, int axis1, int axis2) {
        return Math.sqrt(Math.pow(getJoystickAxis(stick, axis1), 2) + Math.pow(getJoystickAxis(stick, axis2), 2));
    }

    double getDirectionDegrees(int stick, int axis1, int axis2) {
        double x = getJoystickAxis(stick, axis1);
        double y = getJoystickAxis(stick, axis2);
        double angleRad = Math.atan2(y, x);
        double angleDeg = Math.toDegrees(angleRad);
        double finalAng = angleDeg + 90;
        return finalAng;
    }

    double getDriveMagnitude() {
        return getMagnitude(0, 0, 1);
    }

    double getDriveDirectionDegrees() {
        return getDirectionDegrees(0, 0, 1);
    }

    double getFacingJoystickMagnitude() {
        return getMagnitude(0, 4, 5);
    }

    double getFacingJoystickDegrees() {
        return getDirectionDegrees(0, 4, 5);
    } 

    boolean getRGBButtonPressed() {
        return isJoystickButtonPressed(0, 7);
    }

    boolean getAlignWheelsButtonPressed() {
        return isJoystickPovPressed(0, 180);
    }

    boolean getLockButtonPressed() {
        return isJoystickPovPressed(0, 270);
    }

    boolean getDriverResetGyroButtonPressed() {
        return isJoystickPovPressed(0, 90);
    }

    boolean getDriverReverseResetGyroButtonPressed() {
        return isJoystickPovPressed(0, 0);
    }

    boolean getDriveSlowButtonPressed() {
        return isJoystickButtonPressed(0, 6);
    }

    boolean getDriveFastButtonPressed() {
        return isJoystickTriggerPressed(0, 3) ||
                isJoystickTriggerNegativePressed(0, 2);
    }

    boolean getDriveVerySlowButtonPressed() {
        return isJoystickButtonPressed(0, 4);
    }
}