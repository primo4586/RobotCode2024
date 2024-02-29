// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import static frc.robot.Constants.ShooterConstants.*;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.utils.interpolation.InterpolateUtil;
import frc.utils.vision.Vision;

public class ShooterSubsystem extends SubsystemBase {
  private TalonFX m_upShooterMotor;
  private TalonFX m_downShooterMotor;
  public double speed;

  private final Vision vision = Vision.getInstance();

  private final MotionMagicVelocityVoltage motionMagic = new MotionMagicVelocityVoltage(0, MotionMagicAcceleration,
      false, 0.0, 0, false, false, false);

  private static ShooterSubsystem instance;

  NeutralOut neutralOut = new NeutralOut();

  public static ShooterSubsystem getInstance() {
    if (instance == null) {
      instance = new ShooterSubsystem();
    }
    return instance;
  }

  /** Creates a new ShooterSubsysem. */
  private ShooterSubsystem() {
    // giving values to the motors
    this.m_upShooterMotor = new TalonFX(kUpMotorShooterID, Constants.CAN_BUS_NAME);
    this.m_downShooterMotor = new TalonFX(kDownMotorShooterID, Constants.CAN_BUS_NAME);

    // declaring Configs
    TalonFXConfiguration upConfigs = new TalonFXConfiguration();
    TalonFXConfiguration downConfigs = new TalonFXConfiguration();
    MotionMagicConfigs shooterMM = new MotionMagicConfigs();

    // giving motion magic values
    shooterMM.MotionMagicCruiseVelocity = MotionMagicCruiseVelocity;
    shooterMM.MotionMagicAcceleration = MotionMagicAcceleration;
    shooterMM.MotionMagicJerk = MotionMagicJerk;
    upConfigs.MotionMagic = shooterMM;
    downConfigs.MotionMagic = shooterMM;

    // giving PID values
    upConfigs.Slot0.kP = upKP;
    upConfigs.Slot0.kD = upKD;
    upConfigs.Slot0.kS = upKS;
    upConfigs.Slot0.kV = upKV;
    upConfigs.Slot0.kA = upKA;

    downConfigs.Slot0.kP = downKP;
    downConfigs.Slot0.kD = downKD;
    downConfigs.Slot0.kS = downKS;
    downConfigs.Slot0.kV = downKV;
    downConfigs.Slot0.kA = downKA;

    // max voltage for m_shooterMotor
    upConfigs.Voltage.PeakForwardVoltage = PeakForwardVoltage;
    upConfigs.Voltage.PeakReverseVoltage = PeakReverseVoltage;

    downConfigs.Voltage.PeakForwardVoltage = PeakForwardVoltage;
    downConfigs.Voltage.PeakReverseVoltage = PeakReverseVoltage;

    upConfigs.Feedback.SensorToMechanismRatio = GearRatio;
    downConfigs.Feedback.SensorToMechanismRatio = GearRatio;

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

    /* Speed up signals for better charaterization data */
    BaseStatusSignal.setUpdateFrequencyForAll(1000,
        m_upShooterMotor.getVelocity());

    BaseStatusSignal.setUpdateFrequencyForAll(1000,
        m_downShooterMotor.getVelocity());

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
    this.speed = upSpeed;
    this.m_upShooterMotor.setControl(motionMagic.withVelocity(upSpeed));
    this.m_downShooterMotor.setControl(motionMagic.withVelocity(downSpeed));
  }

  // set (active) Shooter motors speed
  public void setShooterSpeed(double Speed) {
    setShooterSpeed(Speed, Speed);
  }

  public void manualSetShooterSpeed(DoubleSupplier speed) {
    this.m_upShooterMotor.set(speed.getAsDouble());
    this.m_downShooterMotor.set(speed.getAsDouble());
  }

  public double getUpShooterSpeed() {
    return m_upShooterMotor.getVelocity().getValue();
  }

  public double getDownShooterSpeed() {
    return m_downShooterMotor.getVelocity().getValue();
  }

  public boolean checkIfShooterAtSpeed() {
    return ((Math.abs(getUpShooterSpeed() - speed) < MaxError)
        && ((Math.abs(getDownShooterSpeed() - speed) < MaxError)));
  }

  public double speakerInterpolate(Pose2d pose2d) {
    return InterpolateUtil.interpolate(ShooterInterpolation, vision.DistanceFromTarget(pose2d));
  }

  public void coast() {
    m_upShooterMotor.setControl(neutralOut);
    m_downShooterMotor.setControl(neutralOut);
  }

  public void UPRunCharacterizationVolts(Double voltage) {
    m_upShooterMotor.setVoltage(voltage);
  }

  public void DownRunCharacterizationVolts(Double voltage) {
    m_downShooterMotor.setVoltage(voltage);
  }

  public double getUPCharacterizationVelocity() {
    return m_upShooterMotor.getVelocity().getValueAsDouble();
  }

  public double getDownCharacterizationVelocity() {
    return m_downShooterMotor.getVelocity().getValueAsDouble();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("lower shooter Speed", getDownShooterSpeed());
    SmartDashboard.putNumber("upper shooter Speed", getUpShooterSpeed());
  }
}