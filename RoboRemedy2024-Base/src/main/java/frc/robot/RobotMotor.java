// RobotMotor.java - Class to be used for any type of motor
package frc.robot;

import edu.wpi.first.wpilibj.*;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import com.revrobotics.*;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.*;

public class RobotMotor {
    public enum RobotMotorType {
        SparkMax, TalonFX, TalonSRX, Victor
    }

    public enum RobotEncoderType {
        None, Internal, Cancoder, MagEncoder, ThroughBore
    }

    private RoboLog rLog;
    private int motorID;
    private RobotMotorType motorType;
    private CANSparkMax neoMotor;
    private RelativeEncoder neoEncoder;
    private SparkMaxPIDController neoPID;
    private TalonFX talonFXMotor;
    private TalonSRX talonSRXMotor;
    private VictorSPX victorMotor;
    private BaseMotorController baseCtreMotor;
    private VoltageOut voltageOut = new VoltageOut(0);
    private PositionVoltage positionOut = new PositionVoltage(0).withSlot(0);
    private VelocityVoltage velocityOut = new VelocityVoltage(0).withSlot(0);
    private Timer timer = new Timer();
    private Timer sparkTimer = new Timer();

    private LinearMapper mapper = null;
    private double unitsPerEncoderTick = 1.0;
    private double encoderTicksPerUnit = 1.0;
    private double unitsPerRevolution = 1.0;
    private double revolutionsPerUnit = 1.0;
    private double lastAssignedPercentage = 0;
    private double lastAssignedPosition = 0;
    private double lastAssignedVelocity = 0;
    private double lastAssignedFeedForward = 0;
    private double lastAssignedTime = 0;
    private boolean isCoast;

    private static int lastFaults = 0;

    RobotMotor(RobotMotorType motorType, int motorID, boolean isInverted, boolean isCoast, RoboLog rLog,
            double unitsPerRevolution, double maxCurrent, RobotEncoderType encoderType, int absoluteEncoderID) {
        this.rLog = rLog;
        this.motorID = motorID;
        this.motorType = motorType;
        this.isCoast = isCoast;
        timer.start();
        sparkTimer.start();
        if (motorType == RobotMotorType.SparkMax) {
            neoMotor = new CANSparkMax(motorID, MotorType.kBrushless);
            neoMotor.restoreFactoryDefaults();
            neoEncoder = neoMotor.getEncoder();
            neoPID = neoMotor.getPIDController();
            neoMotor.setInverted(isInverted);
            neoMotor.setIdleMode(isCoast ? IdleMode.kCoast : IdleMode.kBrake);
            neoEncoder.setVelocityConversionFactor(unitsPerRevolution / 60.0);
            neoEncoder.setPositionConversionFactor(unitsPerRevolution);
            setOutputCurrentLimit(maxCurrent);
            setPID(0, 0, 0, 0, false);
        } else if (motorType == RobotMotorType.TalonFX) {
            talonFXMotor = new TalonFX(motorID);
            TalonFXConfiguration configs = new TalonFXConfiguration(); // Start with Factory Defaults
            configs.MotorOutput.Inverted = (isInverted ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive);
            configs.MotorOutput.NeutralMode = (isCoast ? NeutralModeValue.Coast : NeutralModeValue.Brake);
            configs.MotorOutput.DutyCycleNeutralDeadband = 0.00;
            configs.CurrentLimits.SupplyCurrentLimitEnable = true;
            configs.CurrentLimits.SupplyCurrentLimit = maxCurrent;
            configs.CurrentLimits.SupplyCurrentThreshold = maxCurrent;
            configs.CurrentLimits.SupplyTimeThreshold = 0.2;
            if (encoderType == RobotEncoderType.None) {

            } else if (encoderType == RobotEncoderType.Internal) {
                configs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
            } else if (encoderType == RobotEncoderType.Cancoder) {
                configs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
                configs.Feedback.FeedbackRemoteSensorID = absoluteEncoderID;
            } else {
                rLog.print("Unsupported Encoder Type: " + encoderType);
            }
            talonFXMotor.getConfigurator().apply(configs);
            this.unitsPerRevolution = unitsPerRevolution;
            revolutionsPerUnit = 1.0 / unitsPerRevolution;
        } else if (motorType == RobotMotorType.TalonSRX
                || motorType == RobotMotorType.Victor) {
            if (motorType == RobotMotorType.TalonSRX) {
                talonSRXMotor = new TalonSRX(motorID);
                baseCtreMotor = talonSRXMotor;
            } else if (motorType == RobotMotorType.Victor) {
                victorMotor = new VictorSPX(motorID);
                baseCtreMotor = victorMotor;
            }
            baseCtreMotor.configFactoryDefault();
            baseCtreMotor.setInverted(isInverted);
            baseCtreMotor.setNeutralMode(isCoast ? NeutralMode.Coast : NeutralMode.Brake);
            baseCtreMotor.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_100Ms);
            baseCtreMotor.configVelocityMeasurementWindow(1);
            baseCtreMotor.configNeutralDeadband(0.01);
            baseCtreMotor.configVoltageCompSaturation(12.0);
            baseCtreMotor.enableVoltageCompensation(true);
            setInputCurrentLimit(maxCurrent);
            if (encoderType == RobotEncoderType.None) {

            } else if (encoderType == RobotEncoderType.Internal) {
                baseCtreMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
                unitsPerEncoderTick = unitsPerRevolution / 2048.0;
            } else if (encoderType == RobotEncoderType.Cancoder) {
                baseCtreMotor.configRemoteFeedbackFilter(absoluteEncoderID, RemoteSensorSource.CANCoder, 0);
                baseCtreMotor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);
                unitsPerEncoderTick = unitsPerRevolution / 4096;
            } else if (encoderType == RobotEncoderType.MagEncoder) {
                baseCtreMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
                baseCtreMotor.setSensorPhase(true);
                unitsPerEncoderTick = unitsPerRevolution / 4096;
            } else if (encoderType == RobotEncoderType.ThroughBore) {
                baseCtreMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
                unitsPerEncoderTick = unitsPerRevolution / 8192;
            }
            encoderTicksPerUnit = 1.0 / unitsPerEncoderTick;
        } else {
            this.rLog.print("RobotMotor created with unknown RobotMotorType");
        }
    }

    public int getMotorID() {
        return motorID;
    }

    public void setInputCurrentLimit(double amps) {
        if (motorType == RobotMotorType.TalonFX) {
            CurrentLimitsConfigs config = new CurrentLimitsConfigs();
            talonFXMotor.getConfigurator().refresh(config);
            config.SupplyCurrentLimitEnable = true;
            config.SupplyCurrentLimit = amps;
            config.SupplyCurrentThreshold = amps;
            config.SupplyTimeThreshold = 0.2;
            talonFXMotor.getConfigurator().apply(config);
        } else if (motorType == RobotMotorType.TalonSRX) {
            SupplyCurrentLimitConfiguration talonCurrentLimitConfig = new SupplyCurrentLimitConfiguration(true, amps,
                    amps, 0.2);
            talonSRXMotor.configSupplyCurrentLimit(talonCurrentLimitConfig);
        }
    }

    public void setOutputCurrentLimit(double amps) {
        if (motorType == RobotMotorType.SparkMax) {
            neoMotor.setSmartCurrentLimit((int) amps);
        } else if (motorType == RobotMotorType.TalonFX) {
            CurrentLimitsConfigs falconConfig = new CurrentLimitsConfigs();
            talonFXMotor.getConfigurator().refresh(falconConfig);
            falconConfig.StatorCurrentLimitEnable = true;
            falconConfig.StatorCurrentLimit = amps;
            talonFXMotor.getConfigurator().apply(falconConfig);
        }
    }

    public double getEncoderPosition() {
        if (motorType == RobotMotorType.SparkMax) {
            return neoEncoder.getPosition();
        } else if (motorType == RobotMotorType.TalonFX) {
            return talonFXMotor.getPosition().getValue() * unitsPerEncoderTick;
        } else if (motorType == RobotMotorType.TalonSRX
                || motorType == RobotMotorType.Victor) {
            return baseCtreMotor.getSelectedSensorPosition() * unitsPerEncoderTick;
        } else {
            return 0;
        }
    }

    public double getEncoderVelocity() {
        if (motorType == RobotMotorType.SparkMax) {
            return neoEncoder.getVelocity();
        } else if (motorType == RobotMotorType.TalonFX) {
            return talonFXMotor.getVelocity().getValue() * unitsPerRevolution;
        } else if (motorType == RobotMotorType.TalonSRX
                || motorType == RobotMotorType.Victor) {
            return baseCtreMotor.getSelectedSensorVelocity() * unitsPerEncoderTick * 10;
        } else {
            return 0;
        }
    }

    // Assign the value to be used as the current encoder position
    // This does not move the motor. Use setPosition to move the motor.
    public void setEncoderValue(double position) {
        if (motorType == RobotMotorType.SparkMax) {
            neoEncoder.setPosition(position);
        } else if (motorType == RobotMotorType.TalonFX) {
            talonFXMotor.setPosition(position * encoderTicksPerUnit);
        } else if (motorType == RobotMotorType.TalonSRX
                || motorType == RobotMotorType.Victor) {
            baseCtreMotor.setSelectedSensorPosition((int) (position * encoderTicksPerUnit));
        }
    }

    public void setPercentVelocityLinearMapper(LinearMapper mapper) {
        this.mapper = mapper;
    }

    public LinearMapper getPercentVelocityLinearMapper() {
        if (mapper == null)
            return new LinearMapper();
        else
            return mapper;
    }

    // Assign the PID values to be used for setPosition or setVelicty calls
    public void setPID(double p, double i, double d, double iZone, boolean isVelocity) {
        if (motorType == RobotMotorType.SparkMax) {
            neoPID.setFF(0);
            neoPID.setP(p);
            neoPID.setI(i);
            neoPID.setD(d);
            neoPID.setIZone(iZone);
        } else if (motorType == RobotMotorType.TalonFX) {
            Slot0Configs config = new Slot0Configs();
            talonFXMotor.getConfigurator().refresh(config);
            config.kP = p;
            config.kI = i;
            config.kD = d;
            // TalonFX does not need izone parameter
            talonFXMotor.getConfigurator().apply(config);
        } else if (motorType == RobotMotorType.TalonSRX
                || motorType == RobotMotorType.Victor) {
            double scalingFactor = (isVelocity ? encoderTicksPerUnit * 0.1 : unitsPerEncoderTick);
            baseCtreMotor.config_kF(0, 0);
            baseCtreMotor.config_kP(0, p * scalingFactor);
            baseCtreMotor.config_kI(0, i * scalingFactor);
            baseCtreMotor.config_kD(0, d * scalingFactor);
            baseCtreMotor.config_IntegralZone(0, (int) (iZone * scalingFactor));
        }
    }

    public void setDriveOpenLoopRamp(double rate) {
        if (motorType == RobotMotorType.SparkMax) {
            neoMotor.setOpenLoopRampRate(rate);
        } else if (motorType == RobotMotorType.TalonFX) {
            OpenLoopRampsConfigs config = new OpenLoopRampsConfigs();
            talonFXMotor.getConfigurator().refresh(config);
            config.DutyCycleOpenLoopRampPeriod = rate;
            talonFXMotor.getConfigurator().apply(config);
        } else if (motorType == RobotMotorType.TalonSRX
                || motorType == RobotMotorType.Victor) {
            baseCtreMotor.configOpenloopRamp(rate);
        }
    }

    public void setPercent(double percent) {
        if (motorType == RobotMotorType.SparkMax) {
            neoMotor.set(percent);
        } else if (motorType == RobotMotorType.TalonFX) {
            talonFXMotor.setControl(voltageOut.withOutput(12.0 * percent));
        } else if (motorType == RobotMotorType.TalonSRX
                || motorType == RobotMotorType.Victor) {
            baseCtreMotor.set(ControlMode.PercentOutput, percent);
        }
        lastAssignedPercentage = percent;
        lastAssignedPosition = 0;
        lastAssignedVelocity = 0;
        lastAssignedFeedForward = 0;
        lastAssignedTime = timer.get();
    }

    // Attempt to move the motor to a specific position using a PID loop
    // Use setEncoderValue to change the current encoder value without moving the
    // motor
    public void setPosition(double position) {
        if (motorType == RobotMotorType.SparkMax) {
            neoPID.setReference(position, CANSparkMax.ControlType.kPosition);
            // /rLog.print("Position to Set: " + position);
        } else if (motorType == RobotMotorType.TalonFX) {
            talonFXMotor.setControl(positionOut.withPosition(position));
        } else if (motorType == RobotMotorType.TalonSRX
                || motorType == RobotMotorType.Victor) {
            baseCtreMotor.set(ControlMode.Position, (position * encoderTicksPerUnit));
        }
        lastAssignedPercentage = 0;
        lastAssignedPosition = position;
        lastAssignedVelocity = 0;
        lastAssignedFeedForward = 0;
        lastAssignedTime = timer.get();
    }

    // Attempt to move the motor at a specific velocity using a PID loop
    public void setVelocity(double velocity) {
        double feedForward = getPercentageOutputFromVelocity(velocity);
        if (motorType == RobotMotorType.SparkMax) {
            // rLog.print("setVelocity:" + velocity + " AFF:" + feedForward);
            neoPID.setReference(velocity, CANSparkMax.ControlType.kVelocity, 0, feedForward, ArbFFUnits.kVoltage);
        } else if (motorType == RobotMotorType.TalonFX) {
            if (velocity > 0.01)
            rLog.print("Setting velocity to " + velocity + " using FF of " + feedForward + " voltFF = " +
                feedForward * 12.0);
            talonFXMotor.setControl(velocityOut.withVelocity(velocity * revolutionsPerUnit).withFeedForward(feedForward * 12.0));
        } else if(motorType == RobotMotorType.TalonSRX
                || motorType == RobotMotorType.Victor) {
            baseCtreMotor.set(ControlMode.Velocity, (velocity * encoderTicksPerUnit * 0.1),
                    DemandType.ArbitraryFeedForward, feedForward);
        }
        lastAssignedPercentage = 0;
        lastAssignedPosition = 0;
        lastAssignedVelocity = velocity;
        lastAssignedFeedForward = feedForward;
        lastAssignedTime = timer.get();
    }

    // Calculate the best guess of the voltage perctage output (from -1.0 to 1.0)
    // that
    // will be needed for a given velocity. This value can be used as a feed forward
    // value for a PID loop.
    private double getPercentageOutputFromVelocity(double velocity_In) {
        double percent_Out = 0;
        if (velocity_In >= 0) {
            if (velocity_In < 0.01) {
                percent_Out = 0;
            } else {
                if (mapper != null) {
                    percent_Out = mapper.calculate(velocity_In);
                } else
                    percent_Out = 0;
            }
        } else {
            percent_Out = (-1 * getPercentageOutputFromVelocity(-1 * velocity_In));
        }
        return percent_Out;
    }

    public void setMaxOutput(double percentOutput) {
        if (motorType == RobotMotorType.SparkMax) {
            neoPID.setOutputRange(-percentOutput, percentOutput);
        } else if (motorType == RobotMotorType.TalonFX) {
            VoltageConfigs config = new VoltageConfigs();
            talonFXMotor.getConfigurator().refresh(config);
            config.PeakForwardVoltage = 12.0 * percentOutput;
            config.PeakReverseVoltage = -12.0 * percentOutput;
            talonFXMotor.getConfigurator().apply(config);
        } else if (motorType == RobotMotorType.TalonSRX
                || motorType == RobotMotorType.Victor) {
            baseCtreMotor.configClosedLoopPeakOutput(0, percentOutput);
        }
    }

    // The Spark Max requires this method to be called after changing parameters
    public void burnFlash() {
        if (motorType == RobotMotorType.SparkMax) {
            neoMotor.burnFlash();
        } else if (motorType == RobotMotorType.TalonFX || motorType == RobotMotorType.TalonSRX
                || motorType == RobotMotorType.Victor) {

        }
    }

    public void stopMotor() {
        setPercent(0);
    }

    public double getOutputPercent() {
        double outputPercent = 0;
        if (motorType == RobotMotorType.SparkMax) {
            outputPercent = neoMotor.getAppliedOutput();
        } else if (motorType == RobotMotorType.TalonFX) {
            outputPercent = talonFXMotor.getMotorVoltage().getValue() / 12.0;
        } else if(motorType == RobotMotorType.TalonSRX
                || motorType == RobotMotorType.Victor) {
            outputPercent = baseCtreMotor.getMotorOutputPercent();
        }
        return outputPercent;
    }

    public double getLastAssignedPercentage() {
        return lastAssignedPercentage;
    }

    public double getLastAssignedVelocity() {
        return lastAssignedVelocity;
    }

    public double getLastAssignedPosition() {
        return lastAssignedPosition;
    }

    public double getLastAssignedFeedForward() {
        return lastAssignedFeedForward;
    }

    public void rampToPercent(double percent, double maxPercentPerSecond) {
        double timeBetween = timer.get() - lastAssignedTime;
        double maxChange = maxPercentPerSecond * timeBetween;
        percent = Math.max(Math.min(percent, lastAssignedPercentage + maxChange), lastAssignedPercentage - maxChange);
        setPercent(percent);
    }

    public void rampToVelocity(double velocity, double maxVelocityPerSecond) {
        double timeBetween = timer.get() - lastAssignedTime;
        double maxChange = maxVelocityPerSecond * timeBetween;
        velocity = Math.max(Math.min(velocity, lastAssignedVelocity + maxChange), lastAssignedVelocity - maxChange);
        setVelocity(velocity);
    }

    public void setNeutralModeDeadband(double percentDeadband) {
        if (motorType == RobotMotorType.SparkMax) {
        } else if (motorType == RobotMotorType.TalonFX) {
            MotorOutputConfigs config = new MotorOutputConfigs();
            talonFXMotor.getConfigurator().refresh(config);
            config.DutyCycleNeutralDeadband = percentDeadband;
            talonFXMotor.getConfigurator().apply(config);
        } else if (motorType == RobotMotorType.TalonSRX
                || motorType == RobotMotorType.Victor) {
            baseCtreMotor.configNeutralDeadband(percentDeadband);
        }
    }

    public void setNeutralMode(boolean isCoast) {
        if (this.isCoast == isCoast)
            return;
        this.isCoast = isCoast;
        if (motorType == RobotMotorType.SparkMax) {
            neoMotor.setIdleMode(isCoast ? IdleMode.kCoast : IdleMode.kBrake);
        } else if (motorType == RobotMotorType.TalonFX) {
            MotorOutputConfigs config = new MotorOutputConfigs();
            talonFXMotor.getConfigurator().refresh(config);
            config.NeutralMode = (isCoast ? NeutralModeValue.Coast : NeutralModeValue.Brake);
            talonFXMotor.getConfigurator().apply(config);
        } else if (motorType == RobotMotorType.TalonSRX
                || motorType == RobotMotorType.Victor) {
            baseCtreMotor.setNeutralMode(isCoast ? NeutralMode.Coast : NeutralMode.Brake);
        }
    }

    public void followMotor(RobotMotor motorToFollow) {
        if (motorToFollow.motorType == RobotMotorType.TalonFX && motorType == RobotMotorType.TalonFX) {
            talonFXMotor.setControl(new StrictFollower(motorToFollow.getMotorID()));
        } else if ((motorToFollow.motorType == RobotMotorType.TalonSRX
                    || motorToFollow.motorType == RobotMotorType.Victor)
                && (motorType == RobotMotorType.TalonSRX
                    || motorType == RobotMotorType.Victor)) {
            baseCtreMotor.follow(motorToFollow.baseCtreMotor);
        } else {
            rLog.print("Can not follow Motor type" + motorType + " " + motorToFollow.motorType);
        }
    }

    public void setIntegralAccumulator(double value) {
        if (motorType == RobotMotorType.SparkMax) {
            neoPID.setIAccum(value);
        } else if (motorType == RobotMotorType.TalonFX) {
            // TalonFX does not need to use integral accumulator
        } else if (motorType == RobotMotorType.TalonSRX
                || motorType == RobotMotorType.Victor) {
            baseCtreMotor.setIntegralAccumulator(value);
        }
    }

    public boolean isAttached() {
        if (motorType == RobotMotorType.SparkMax) {
            if (sparkTimer.get() >= 1) {
                int faults = neoMotor.getFaults();
                if (faults != 0)
                    lastFaults = faults;
                sparkTimer.reset();
                sparkTimer.start();
            }
            return (lastFaults == 0);
        } else if (motorType == RobotMotorType.TalonFX) {
            return talonFXMotor.isAlive();
        } else if (motorType == RobotMotorType.TalonSRX
                || motorType == RobotMotorType.Victor) {
            if (baseCtreMotor.getLastError() != ErrorCode.CAN_MSG_NOT_FOUND) {
                return true;
            }
        }
        return false;
    }
}