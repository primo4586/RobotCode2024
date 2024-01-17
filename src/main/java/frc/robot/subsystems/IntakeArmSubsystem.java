// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import static frc.robot.Constants.IntakeArm.*;

public class IntakeArmSubsystem extends SubsystemBase {
  private TalonFX m_IntakeArmMotor;
  private final MotionMagicVoltage motionMagic = new MotionMagicVoltage(0);
  private final DigitalInput intakeSwitch = new DigitalInput(switchID);
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
    this.m_IntakeArmMotor = new TalonFX(KMotorID);
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
    configs.Feedback.SensorToMechanismRatio = SensorToMechanismRatio;

    configs.SoftwareLimitSwitch.ForwardSoftLimitEnable = ForwardSoftLimitEnable;
    configs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ForwardSoftLimitThreshold;
    configs.SoftwareLimitSwitch.ReverseSoftLimitEnable = ReverseSoftLimitEnable;
    configs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = RevesrseSoftLimitThreshold;


    // gives code to TalonFX
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; i++) {
      status = m_IntakeArmMotor.getConfigurator().apply(configs);
      if (status.isOK()) {
        break;
      }
    }
    if (!status.isOK()) {
      System.out.println("Turret could not apply configs, error code: " + status.toString());
    }

    // make sure we start at 0.
    m_IntakeArmMotor.setPosition(0);
  }

  // moving the arm
  public void moveArmTo(double degrees){
      m_IntakeArmMotor.setControl(motionMagic.withPosition(degrees));
  }
  //set speed
  public void setSpeed(double speed){
    m_IntakeArmMotor.set(speed);
  }
  // set incoder idk
  public void setEncoder (double incoder){
    m_IntakeArmMotor.setPosition(incoder);
  }
  // get if a switch is press
  public boolean getSwitch(){
    return intakeSwitch.get();
  }
  // check if intake arm is in place 
  public boolean checkIntakeArmPosion(){
    return Math.abs(m_IntakeArmMotor.getClosedLoopError().getValue()) < minimumError;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
