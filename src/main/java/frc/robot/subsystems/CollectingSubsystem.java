// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.CollectingConstants.*;

public class CollectingSubsystem extends SubsystemBase {
  private TalonSRX m_collecting;
  DigitalInput limitSwitch;

  private static CollectingSubsystem instance;
  
    public static CollectingSubsystem getInstance() {
      if (instance == null) {
        instance = new CollectingSubsystem();
      }
      return instance;
    }
    
  /** Creates a new CollectingSubsystem. */
  public CollectingSubsystem() {
    this.m_collecting = new TalonSRX(CollectingMotorID);
    this.limitSwitch = new DigitalInput(SwitchID);
  }

  public void setSpeed(double speed){
    m_collecting.set(ControlMode.PercentOutput, speed);
  }

  public boolean getSwitch(){
    return limitSwitch.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
