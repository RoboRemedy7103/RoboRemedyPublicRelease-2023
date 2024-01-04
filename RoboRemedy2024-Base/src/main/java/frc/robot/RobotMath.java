// RobotMath.java - Common math functions
package frc.robot;

public class RobotMath {
    public static double angleCenteredOnTarget(double angle, double goalAngle) {
        while (angle < (goalAngle - 180)) {
            angle += 360;
        }
        while (angle > (goalAngle + 180)) {
            angle -= 360;
        }

        return angle;
    }

    public static double angleDifferenceFromTarget(double angle, double goalAngle) {
        double difference = goalAngle - angle;
        return angleCenteredOnTarget(difference, 0);
    }

    // Round a double to 1 decimal place
    static double round1(double valueToRound) {
        return Math.round(valueToRound * 10.0) / 10.0;
    }

    // Round a double to 2 decimal places
    public static double round2(double valueToRound) {
        return Math.round(valueToRound * 100.00) / 100.00;
    }

    public static double pythagoreanTheorem(double leg1, double leg2) {
        return Math.sqrt((leg1 * leg1) + (leg2 * leg2));
    }

    // Impliments minMax
    static double minMax(double minValue, double maxValue, double Value) {
        return Math.min(Math.max(minValue, Value), maxValue);
    }
}