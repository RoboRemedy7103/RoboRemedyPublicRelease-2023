// RobotEnums.java - Enums used by robot project
package frc.robot;

public class RobotEnums {

    public enum RobotName {
        LIVE2024, JerryJr, Scorpio
    }

    public enum AutoProgram {
        JustAlign,
        DriveOut,
        None
    }

    public enum TestMode {
        QuickTest,
        SwerveTest,
        SwerveDrive,
        TestAngle,
        TestMotors,
        TuneMotor,
        TestLED,
        ResetEncoders,
        TestCountdown,
        TestNone
    }

    public enum LEDregion {
        regionOne,
        regionTwo
    }

    public enum LEDcolor {
        BadRed,
        Red,
        DimRed,
        Green,
        GreenBlue,
        Blue,
        DimBlue,
        Yellow,
        Black,
        Purple,
        White,
        DimWhite,
        Orange
    }
}
