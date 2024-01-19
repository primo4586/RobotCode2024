// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import static frc.robot.Constants.TrapArmConstants.*;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TrapArmSubsystem extends SubsystemBase {
  private TalonSRX m_TrapMotor;
  DigitalInput innerSwitch;
  DigitalInput outerSwitch;

  private static TrapArmSubsystem instance;

  public static TrapArmSubsystem getInstance() {
    if (instance == null) {
      instance = new TrapArmSubsystem();
    }
    return instance;
  }
  
  /** Creates a new TrapArmSubsystem. */
  public TrapArmSubsystem() {
    this.m_TrapMotor = new TalonSRX(ArmMotorID);
    this.innerSwitch = new DigitalInput(InnerSwitchID);
    this.outerSwitch = new DigitalInput(OuterSwitchID);
  }


  public void setSpeed(double speed){
    m_TrapMotor.set(ControlMode.PercentOutput,speed);
  }


  public boolean getInnerSwitch(){
    return innerSwitch.get();
  }


  public boolean getOuterSwitch(){
    return outerSwitch.get();
  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
