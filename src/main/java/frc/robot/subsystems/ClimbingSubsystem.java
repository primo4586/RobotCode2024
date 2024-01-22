// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.climbingConstants.*;



public class ClimbingSubsystem extends SubsystemBase {
  /** Creates a new ClimbingSubsystem. */
  private CANSparkMax m_climbingRight;
  private CANSparkMax m_climbingLeft;
  double speed;

  private static ClimbingSubsystem instance;
  
  public static ClimbingSubsystem getInstance()
  {
   if (instance == null)
    {
      instance = new ClimbingSubsystem();
    }
    return instance;
  }
  

  private ClimbingSubsystem() 
  {
    m_climbingLeft = new CANSparkMax(M_CLIMBINGLEFT_MOTOR_ID, MotorType.kBrushless);
    m_climbingRight = new CANSparkMax(M_CLIMBINGRIGHT_MOTOR_ID, MotorType.kBrushless);



  }
  public void setSpeedClimbing(double speed){
    m_climbingLeft.set(speed);
    m_climbingRight.set(speed);
  }

  public void setRightSpeed(double speed){
    m_climbingRight.set(speed);
  }
  public void setLeftSpeed(double speed)
  {
    m_climbingLeft.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
