// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SubsystemTrap extends SubsystemBase {
  private WPI_TalonSRX trapMotor;
  double speed;
   private static SubsystemTrap instance;
  
  public static SubsystemTrap getInstance()
  {
   if (instance == null)
    {
      instance = new SubsystemTrap();
    }
    return instance;
  }


  /** Creates a new SubsystemTrap. */
  public SubsystemTrap() {

  }
  public void setSpeed(double speed){
    trapMotor.set(speed);
    
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
