package frc.Utils.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;

import edu.wpi.first.math.util.Units;
import frc.Utils.motors.FalconConversions;
import frc.robot.Constants;

public final class CTREConfigs {
    public TalonFXConfiguration swerveDriveFXConfig = new TalonFXConfiguration();
    public CANcoderConfiguration swerveCANcoderConfig = new CANcoderConfiguration();

    public CTREConfigs() {

        /** Swerve Drive Motor Configuration */
        /* Motor Inverts and Neutral Mode */
        var driveMotorOutput = swerveDriveFXConfig.MotorOutput;
        driveMotorOutput.Inverted = Constants.Swerve.driveMotorInvert;
        driveMotorOutput.NeutralMode = Constants.Swerve.driveNeutralMode;

        //MM
        var driveMotionMagic = swerveDriveFXConfig.MotionMagic;
        driveMotionMagic.MotionMagicAcceleration = FalconConversions.MPSToTalon(7, Units.inchesToMeters(4),
                Constants.Swerve.driveGearRatio);
        driveMotionMagic.MotionMagicCruiseVelocity = FalconConversions.MPSToTalon(3.5, Units.inchesToMeters(4),
                Constants.Swerve.driveGearRatio);
        driveMotionMagic.MotionMagicJerk = driveMotionMagic.MotionMagicAcceleration * 10;

        /* Current Limiting */
        var driveCurrentLimits = swerveDriveFXConfig.CurrentLimits;
        driveCurrentLimits.SupplyCurrentLimitEnable = Constants.Swerve.driveEnableCurrentLimit;
        driveCurrentLimits.SupplyCurrentLimit = Constants.Swerve.driveCurrentLimit;
        driveCurrentLimits.SupplyCurrentThreshold = Constants.Swerve.driveCurrentThreshold;
        driveCurrentLimits.SupplyTimeThreshold = Constants.Swerve.driveCurrentThresholdTime;

        /* PID Config */
        var driveSlot0 = swerveDriveFXConfig.Slot0;
        driveSlot0.kP = 0.2;
        driveSlot0.kI = 0;
        driveSlot0.kD = 0.0;
        driveSlot0.kS = 0.10502;
        driveSlot0.kV = 0.1082;
        driveSlot0.kA = 0.01516;


        /* Open and Closed Loop Ramping */
        swerveDriveFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Constants.Swerve.openLoopRamp;
        swerveDriveFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = Constants.Swerve.openLoopRamp;

        swerveDriveFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = Constants.Swerve.closedLoopRamp;
        swerveDriveFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.Swerve.closedLoopRamp;

        /** Swerve CANCoder Configuration */
        swerveCANcoderConfig.MagnetSensor.SensorDirection = Constants.Swerve.cancoderInvert;
        swerveCANcoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
    }
}