// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import static frc.robot.Constants.ShooterConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Utils.interpolation.InterpolateUtil;
import frc.Utils.vision.Vision;

public class ShooterSubsysem extends SubsystemBase {
  private TalonFX m_shooterMotor;
  private final Vision vision = Vision.getInstance();

  private final MotionMagicVelocityVoltage motionMagic = new MotionMagicVelocityVoltage(0);

  private static ShooterSubsysem instance;

  public static ShooterSubsysem getInstance() {
    if (instance == null) {
      instance = new ShooterSubsysem();
    }
    return instance;
  }

  /** Creates a new ShooterSubsysem. */
  private ShooterSubsysem() {
    // giving values to the motors
    this.m_shooterMotor = new TalonFX(kMotorShooterID);

    // declaring Configs
    TalonFXConfiguration configs = new TalonFXConfiguration();
    MotionMagicConfigs shooterMM = new MotionMagicConfigs();

    // giving motion magic values
    shooterMM.MotionMagicCruiseVelocity = MotionMagicCruiseVelocity;
    shooterMM.MotionMagicAcceleration = MotionMagicAcceleration;
    shooterMM.MotionMagicJerk = MotionMagicJerk;
    configs.MotionMagic = shooterMM;

    // giving PID values
    configs.Slot0.kP = kP;
    configs.Slot0.kD = kD;
    configs.Slot0.kS = kS;
    configs.Slot0.kV = kV;

    // max voltage for m_shooterMotor
    configs.Voltage.PeakForwardVoltage = PeakForwardVoltage;
    configs.Voltage.PeakReverseVoltage = PeakReverseVoltage;

    configs.Feedback.SensorToMechanismRatio = SensorToMechanismRatio;

    // Checking if m_shooterMotor apply configs
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = m_shooterMotor.getConfigurator().apply(configs);
      if (status.isOK())
        break;
    }
    if (!status.isOK()) {
      System.out.println("Shooter could not apply configs, error code " + status.toString());
    }
  }

  // set (active) shooter motor speed
  public void setShooterSpeed(double speed) {
    this.m_shooterMotor.setControl(motionMagic.withVelocity(speed));
  }

  public double getShooterSpeed() {
    return m_shooterMotor.getVelocity().getValue();
  }

  public boolean checkIfShooterAtSpeed() {
    return (Math.abs(m_shooterMotor.getClosedLoopError().getValue()) < MaxError);
  }

  public double InterpolationValue(Pose2d pose2d) {
    return InterpolateUtil.interpolate(ShooterInterpolation, vision.DistanceFromTarget(pose2d));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}