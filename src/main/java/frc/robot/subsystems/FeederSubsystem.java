// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.FeederConstants.*;

public class FeederSubsystem extends SubsystemBase {
  /** Creates a new FeederSubsystem. */

  //the feeder's motor
  TalonSRX m_Feeder = new TalonSRX(FeederMotorId);

  // singelton
  public static FeederSubsystem instance;

  public static FeederSubsystem getInstance() {
    if (instance == null) {
        instance = new FeederSubsystem();
    }
    return instance;
  } 

  // set speed function
  public void setSpeed(double MotorSpeed){

    MotorSpeed = FeederMotorSpeed;
    m_Feeder.set(ControlMode.PercentOutput, MotorSpeed, DemandType.Neutral, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
