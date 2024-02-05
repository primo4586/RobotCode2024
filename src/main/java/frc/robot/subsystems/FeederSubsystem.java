// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.FeederConstants.*;

public class FeederSubsystem extends SubsystemBase {
  /** Creates a new FeederSubsystem. */

  // the feeder's motor
  CANSparkMax m_Feeder;
  DigitalInput m_feederNoteSensor;

  // singelton
  private static FeederSubsystem instance;

  public static FeederSubsystem getInstance() {
    if (instance == null) {
      instance = new FeederSubsystem();
    }
    return instance;
  }

  // constructor
  private FeederSubsystem() {
    m_Feeder = new CANSparkMax(FeederMotorId, MotorType.kBrushless);
    this.m_feederNoteSensor = new DigitalInput(feederNoteSensorID);
  }

  // set speed function
  public void setSpeed(double motorSpeed) {
    m_Feeder.set(motorSpeed);
  }

  public boolean getSwitch() {
    return m_feederNoteSensor.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
