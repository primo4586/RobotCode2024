// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.Constants.IntakeArmConstants.*;

import java.util.function.DoubleSupplier;

public class IntakeArmSubsystem extends SubsystemBase {
  private TalonFX m_IntakeArmMotor;
  private final MotionMagicVoltage motionMagic = new MotionMagicVoltage(intakeArmStartingValue);
  private final DigitalInput intakeArmSwitch = new DigitalInput(intakeArmSwitchID);

  // singleton
  private static IntakeArmSubsystem instance;

  public static IntakeArmSubsystem getInstance() {
    if (instance == null) {
      instance = new IntakeArmSubsystem();
    }
    return instance;
  }

  /** Creates a new IntakeArmSubsystem. */
  private IntakeArmSubsystem() {
    this.m_IntakeArmMotor = new TalonFX(IntakeArmMotorID, Constants.canBus_name);
    TalonFXConfiguration configs = new TalonFXConfiguration();
    MotionMagicConfigs mm = new MotionMagicConfigs();

    mm.MotionMagicCruiseVelocity = mmVelocity;
    mm.MotionMagicAcceleration = mmAcceleration;
    mm.MotionMagicJerk = mmJerk;
    configs.MotionMagic = mm;

    configs.Slot0.kP = KP;
    configs.Slot0.kD = KD;
    configs.Slot0.kV = KV;
    configs.Slot0.kS = KS;

    configs.Voltage.PeakForwardVoltage = PeakForwardVoltage;
    configs.Voltage.PeakReverseVoltage = PeakReverseVoltage;
    configs.Feedback.SensorToMechanismRatio = TICKS_PER_DEGREE;

    configs.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    configs.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

    // configs.SoftwareLimitSwitch.ForwardSoftLimitEnable = ForwardSoftLimitEnable;
    // configs.SoftwareLimitSwitch.ReverseSoftLimitEnable = ReverseSoftLimitEnable;
    configs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ForwardSoftLimitThreshold;
    configs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = RevesrseSoftLimitThreshold;

    configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // gives code to TalonFX
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; i++) {
      status = m_IntakeArmMotor.getConfigurator().apply(configs);
      if (status.isOK()) {
        break;
      }
    }
    if (!status.isOK()) {
      System.out.println("intake arm could not apply configs, error code: " + status.toString());
    }

    m_IntakeArmMotor.setPosition(intakeArmStartingValue);
  }

  // moving the arm
  public void moveArmTo(double degrees) {
    m_IntakeArmMotor.setControl(motionMagic.withPosition(degrees));
  }

  // set speed
  public void setSpeed(DoubleSupplier speed) {
    m_IntakeArmMotor.set(speed.getAsDouble());
  }

  // set encoder
  public void setEncoder(double encoderValue) {
    m_IntakeArmMotor.setPosition(encoderValue);
  }

  // get if a switch is press
  public boolean getSwitch() {
    return !intakeArmSwitch.get();
  }

  // check if intake arm is in place
  public boolean checkIntakeArmPosion() {
    return Math.abs(m_IntakeArmMotor.getClosedLoopError().getValue()) < minimumError;
  }

  public void runCharacterizationVolts(Double voltage) {
    m_IntakeArmMotor.setVoltage(voltage);
  }

  public double getCharacterizationVelocity() {
    return m_IntakeArmMotor.getVelocity().getValueAsDouble();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putBoolean("intkeArmSwitch", getSwitch());
    SmartDashboard.putNumber("intakeArm pose", m_IntakeArmMotor.getPosition().getValueAsDouble());
  }
}
