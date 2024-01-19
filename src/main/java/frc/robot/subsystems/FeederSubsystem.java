// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.fasterxml.jackson.annotation.JsonCreator.Mode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.FeederConstants.*;

public class FeederSubsystem extends SubsystemBase {
  /** Creates a new FeederSubsystem. */

  //the feeder's motor
  TalonSRX m_Feeder;

  //constructor
  private FeederSubsystem(){
    m_Feeder = new TalonSRX(FeederMotorId);
  }

  // singelton
  private static FeederSubsystem instance;

  public static FeederSubsystem getInstance() {
    if (instance == null) {
        instance = new FeederSubsystem();  
    }
    return instance;
  } 

  // set speed function
  public void setSpeed(double motorSpeed){
    m_Feeder.set(ControlMode.PercentOutput, motorSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
