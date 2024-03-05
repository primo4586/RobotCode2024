package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import static frc.robot.subsystems.shooter.ShooterConstants.shooterConstants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.util.AllianceFlipUtil;
import frc.util.interpolation.InterpolateUtil;

public class ShooterSubsystem extends SubsystemBase {
    private TalonFX m_upShooterMotor;
    private TalonFX m_downShooterMotor;
    private double targetSpeed;
    private final SwerveSubsystem swerve = SwerveSubsystem.getInstance();

    private final MotionMagicVelocityVoltage motionMagic = new MotionMagicVelocityVoltage(0,
            shooterConstants.MOTION_MAGIC_ACCELERATION,
            false, 0.0, 0, false, false, false);

    private static ShooterSubsystem instance;

    NeutralOut neutralOut = new NeutralOut();

    public static ShooterSubsystem getInstance() {
        if (instance == null) {
            instance = new ShooterSubsystem();
        }
        return instance;
    }

    /** Creates a new ShooterSubsystem. */
    private ShooterSubsystem() {
        // giving values to the motors
        this.m_upShooterMotor = new TalonFX(shooterConstants.UP_MOTOR_SHOOTER_ID, Constants.CAN_BUS_NAME);
        this.m_downShooterMotor = new TalonFX(shooterConstants.DOWN_MOTOR_SHOOTER_ID, Constants.CAN_BUS_NAME);

        // declaring Configs
        TalonFXConfiguration upConfigs = new TalonFXConfiguration();
        TalonFXConfiguration downConfigs = new TalonFXConfiguration();
        MotionMagicConfigs shooterMM = new MotionMagicConfigs();

        // giving motion magic values
        shooterMM.MotionMagicCruiseVelocity = shooterConstants.MOTION_MAGIC_CRUISE_VELOCITY;
        shooterMM.MotionMagicAcceleration = shooterConstants.MOTION_MAGIC_ACCELERATION;
        shooterMM.MotionMagicJerk = shooterConstants.MOTION_MAGIC_JERK;
        upConfigs.MotionMagic = shooterMM;
        downConfigs.MotionMagic = shooterMM;

        // giving PID values
        upConfigs.Slot0.kP = shooterConstants.UP_KP;
        upConfigs.Slot0.kD = shooterConstants.UP_KD;
        upConfigs.Slot0.kS = shooterConstants.UP_KS;
        upConfigs.Slot0.kV = shooterConstants.UP_KV;
        upConfigs.Slot0.kA = shooterConstants.UP_KA;

        downConfigs.Slot0.kP = shooterConstants.DOWN_KP;
        downConfigs.Slot0.kD = shooterConstants.DOWN_KD;
        downConfigs.Slot0.kS = shooterConstants.DOWN_KS;
        downConfigs.Slot0.kV = shooterConstants.DOWN_KV;
        downConfigs.Slot0.kA = shooterConstants.DOWN_KA;

        // max voltage for m_shooterMotor
        upConfigs.Voltage.PeakForwardVoltage = shooterConstants.PEAK_FORWARD_VOLTAGE;
        upConfigs.Voltage.PeakReverseVoltage = shooterConstants.PEAK_REVERSE_VOLTAGE;

        downConfigs.Voltage.PeakForwardVoltage = shooterConstants.PEAK_FORWARD_VOLTAGE;
        downConfigs.Voltage.PeakReverseVoltage = shooterConstants.PEAK_REVERSE_VOLTAGE;

        upConfigs.Feedback.SensorToMechanismRatio = shooterConstants.GEAR_RATIO;
        downConfigs.Feedback.SensorToMechanismRatio = shooterConstants.GEAR_RATIO;

        upConfigs.CurrentLimits.SupplyCurrentLimit = 40;
        upConfigs.CurrentLimits.SupplyCurrentThreshold = 50;
        upConfigs.CurrentLimits.SupplyTimeThreshold = 0.1;
        upConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

        downConfigs.CurrentLimits.SupplyCurrentLimit = 40;
        downConfigs.CurrentLimits.SupplyCurrentThreshold = 50;
        downConfigs.CurrentLimits.SupplyTimeThreshold = 0.1;
        downConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

        m_upShooterMotor.setNeutralMode(NeutralModeValue.Coast);
        m_downShooterMotor.setNeutralMode(NeutralModeValue.Coast);

        upConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        downConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        /* Speed up signals for better characterization data */
        BaseStatusSignal.setUpdateFrequencyForAll(1000, m_upShooterMotor.getVelocity());
        BaseStatusSignal.setUpdateFrequencyForAll(1000, m_downShooterMotor.getVelocity());

        m_upShooterMotor.optimizeBusUtilization();
        m_downShooterMotor.optimizeBusUtilization();

        // Checking if m_upShooterMotor apply configs
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = m_upShooterMotor.getConfigurator().apply(upConfigs);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println("Shooter UP could not apply upConfigs, error code " + status.toString());
        }

        // Checking if m_downShooterMotor apply configs
        for (int i = 0; i < 5; ++i) {
            status = m_downShooterMotor.getConfigurator().apply(downConfigs);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println("Shooter could not apply downConfigs, error code " + status.toString());
        }
    }

    // set (active) Shooter motors speed
    public void setShooterSpeed(double upSpeed, double downSpeed) {
        this.targetSpeed = upSpeed;
        this.m_upShooterMotor.setControl(motionMagic.withVelocity(upSpeed));
        this.m_downShooterMotor.setControl(motionMagic.withVelocity(downSpeed));
    }

    // set (active) Shooter motors speed
    public void setShooterSpeed(double speed) {
        this.targetSpeed = speed;
        setShooterSpeed(speed, speed);
    }

    public void manualSetShooterSpeed(double speed) {
        this.m_upShooterMotor.set(speed);
        this.m_downShooterMotor.set(speed);
    }

    public double getUpShooterSpeed() {
        return m_upShooterMotor.getVelocity().getValue();
    }

    public double getDownShooterSpeed() {
        return m_downShooterMotor.getVelocity().getValue();
    }

    public boolean checkIfShooterAtSpeed() {
        return ((Math.abs(getUpShooterSpeed() - targetSpeed) < shooterConstants.MAX_ERROR)
                && ((Math.abs(getDownShooterSpeed() - targetSpeed) < shooterConstants.MAX_ERROR)));
    }

    public double speakerInterpolate() {
        return InterpolateUtil.interpolate(
                shooterConstants.SHOOTER_INTERPOLATION,
                swerve.getPose().getTranslation().getDistance(
                        AllianceFlipUtil.apply(FieldConstants.Speaker.centerSpeakerOpening).getTranslation()));
    }

    public void coast() {
        m_upShooterMotor.setControl(neutralOut);
        m_downShooterMotor.setControl(neutralOut);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // SmartDashboard.putNumber("uper shooter heat", m_upShooterMotor.getDeviceTemp().getValueAsDouble());
        // SmartDashboard.putNumber("lower shooter heat", m_downShooterMotor.getDeviceTemp().getValueAsDouble());
        // SmartDashboard.putNumber("uper shooter current", m_upShooterMotor.getSupplyCurrent().getValueAsDouble());
        // SmartDashboard.putNumber("lower shooter current", m_downShooterMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("lower shooter Speed", getDownShooterSpeed());
        SmartDashboard.putNumber("upper shooter Speed", getUpShooterSpeed());

        SmartDashboard.putNumber("up error", getUpShooterSpeed()-70);
        SmartDashboard.putNumber("down error", getDownShooterSpeed()-70);

        SmartDashboard.putBoolean("shooter ready", Math.abs(getDownShooterSpeed()-getUpShooterSpeed())<0.3);
    }
}
