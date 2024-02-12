// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import static frc.robot.Constants.ShooterConstants.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Utils.interpolation.InterpolateUtil;
import frc.Utils.vision.Vision;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  private TalonFX m_upShooterMotor;
  private TalonFX m_downShooterMotor;

  private final Vision vision = Vision.getInstance();

  private final MotionMagicVelocityVoltage motionMagic = new MotionMagicVelocityVoltage(0);

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
    this.m_upShooterMotor = new TalonFX(kUpMotorShooterID, Constants.canBus_name);
    this.m_downShooterMotor = new TalonFX(kDownMotorShooterID, Constants.canBus_name);


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

    downConfigs.Slot0.kP = downKP;
    downConfigs.Slot0.kD = downKD;
    downConfigs.Slot0.kS = downKS;
    downConfigs.Slot0.kV = downKV;

    // max voltage for m_shooterMotor
    upConfigs.Voltage.PeakForwardVoltage = PeakForwardVoltage;
    upConfigs.Voltage.PeakReverseVoltage = PeakReverseVoltage;

    downConfigs.Voltage.PeakForwardVoltage = PeakForwardVoltage;
    downConfigs.Voltage.PeakReverseVoltage = PeakReverseVoltage;

    upConfigs.Feedback.SensorToMechanismRatio = SensorToMechanismRatio;
    downConfigs.Feedback.SensorToMechanismRatio = SensorToMechanismRatio;
    
    m_upShooterMotor.setNeutralMode(NeutralModeValue.Coast);
    m_downShooterMotor.setNeutralMode(NeutralModeValue.Coast);

    upConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    downConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

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
  public void setShooterSpeed(double upSpeed,double downSpeed) {
    this.m_upShooterMotor.setControl(motionMagic.withVelocity(upSpeed));
    this.m_downShooterMotor.setControl(motionMagic.withVelocity(downSpeed));

  }

  public double getUpShooterSpeed() {
    return m_upShooterMotor.getVelocity().getValue();
  }

    public double getDownShooterSpeed() {
    return m_downShooterMotor.getVelocity().getValue();
  }

  public boolean checkIfShooterAtSpeed() {
    return ((Math.abs(m_upShooterMotor.getClosedLoopError().getValue()) < MaxError) && ((Math.abs(m_downShooterMotor.getClosedLoopError().getValue()) < MaxError)));
  }

  public double speakerInterpolate(Pose2d pose2d) {
    return InterpolateUtil.interpolate(ShooterInterpolation, vision.DistanceFromTarget(pose2d));
  }

  public void coast() {
    m_upShooterMotor.setControl(neutralOut);
    m_downShooterMotor.setControl(neutralOut);
  }

  public void runCharacterizationVolts(Double upVoltage,double downVoltage) {
    m_upShooterMotor.setVoltage(upVoltage);
    m_downShooterMotor.setVoltage(downVoltage);
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
  }
}