// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climb.ClimbConstants.climbingConstants;

public class ClimbingSubsystem extends SubsystemBase {

  private CANSparkMax m_motorRight;
  private CANSparkMax m_motorLeft;

  private ClimbingSubsystem() {}

  @Override
  public void periodic() {
    m_motorRight = new CANSparkMax(climbingConstants.M_CLIMBINGRIGHT_MOTOR_ID, MotorType.kBrushless);
    m_motorLeft = new CANSparkMax(climbingConstants.M_CLIMBINGLEFT_MOTOR_ID, MotorType.kBrushless);

    m_motorRight.setInverted(false);
    m_motorLeft.setInverted(true);
  }

  public void setSpeed(double speed) {
    m_motorRight.set(speed);
    m_motorLeft.set(speed);
  }

  public void setRightSpeed(double speed) {
    m_motorRight.set(speed);
  }

  public void setLeftSpeed(double speed) {
    m_motorLeft.set(speed);
  }

  private static ClimbingSubsystem instance;

  public static ClimbingSubsystem getInstance() {
    if (instance == null) {
      instance = new ClimbingSubsystem();
    }
    return instance;
  }
}
