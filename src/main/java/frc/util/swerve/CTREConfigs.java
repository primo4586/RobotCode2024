package frc.util.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.swerve.SwerveConstants.swerveConstants;
import frc.util.motors.FalconConversions;

public final class CTREConfigs {
    public TalonFXConfiguration swerveDriveFXConfig = new TalonFXConfiguration();
    public CANcoderConfiguration swerveCANcoderConfig = new CANcoderConfiguration();

    public CTREConfigs() {

        /** Swerve Drive Motor Configuration */
        /* Motor Inverts and Neutral Mode */
        var driveMotorOutput = swerveDriveFXConfig.MotorOutput;
        driveMotorOutput.Inverted = swerveConstants.driveMotorInvert;
        driveMotorOutput.NeutralMode = swerveConstants.driveNeutralMode;

        //MM
        var driveMotionMagic = swerveDriveFXConfig.MotionMagic;
        driveMotionMagic.MotionMagicAcceleration = FalconConversions.MPSToTalon(7, Units.inchesToMeters(4),
                swerveConstants.driveGearRatio);
        driveMotionMagic.MotionMagicCruiseVelocity = FalconConversions.MPSToTalon(3.5, Units.inchesToMeters(4),
                swerveConstants.driveGearRatio);
        driveMotionMagic.MotionMagicJerk = driveMotionMagic.MotionMagicAcceleration * 10;

        /* Current Limiting */
        var driveCurrentLimits = swerveDriveFXConfig.CurrentLimits;
        driveCurrentLimits.SupplyCurrentLimitEnable = swerveConstants.driveEnableCurrentLimit;
        driveCurrentLimits.SupplyCurrentLimit = swerveConstants.driveCurrentLimit;
        driveCurrentLimits.SupplyCurrentThreshold = swerveConstants.driveCurrentThreshold;
        driveCurrentLimits.SupplyTimeThreshold = swerveConstants.driveCurrentThresholdTime;

        /* PID Config */
        var driveSlot0 = swerveDriveFXConfig.Slot0;
        driveSlot0.kP = 0.2;
        driveSlot0.kI = 0;
        driveSlot0.kD = 0.0;
        driveSlot0.kS = 0.10502;
        driveSlot0.kV = 0.1082;
        driveSlot0.kA = 0.01516;


        /* Open and Closed Loop Ramping */
        swerveDriveFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = swerveConstants.openLoopRamp;
        swerveDriveFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = swerveConstants.openLoopRamp;

        swerveDriveFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = swerveConstants.closedLoopRamp;
        swerveDriveFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = swerveConstants.closedLoopRamp;

        /** Swerve CANCoder Configuration */
        swerveCANcoderConfig.MagnetSensor.SensorDirection = swerveConstants.cancoderInvert;
        swerveCANcoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
    }
}