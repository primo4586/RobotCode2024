// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.subsystems.climb.climbConstants.*;

import java.util.function.DoubleSupplier;

/**
 * The ClimbSubsystem is a class that controls the two motors on the
 * climb subsystem.
 */
public class ClimbSubsystem extends SubsystemBase {
  private final CANSparkMax m_RightSparkMax = new CANSparkMax(M_CLIMB_RIGHT_MOTOR_ID, MotorType.kBrushless);
  private final CANSparkMax m_LeftSparkMax = new CANSparkMax(M_CLIMB_LEFT_MOTOR_ID, MotorType.kBrushless);

  private static ClimbSubsystem INSTANCE;

  /**
   * Gets the ClimbSubsystem instance. If it hasn't been created yet,
   * create it.
   * 
   * @return the ClimbSubsystem instance
   */
  public static ClimbSubsystem getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new ClimbSubsystem();
    }
    return INSTANCE;
  }

  /**
   * Creates a new ClimbSubsystem.
   */
  private ClimbSubsystem() {
    m_LeftSparkMax.follow(m_RightSparkMax, true);
  }

  /**
   * Creates a command that makes the climb motors move.
   * 
   * @param speed the speed to move the motors at
   * @return the command
   */
  public Command moveClimb(DoubleSupplier speed) {
    return run(() -> m_RightSparkMax.set(speed.getAsDouble()));
  }

  /**
   * Creates a command that makes the climb motors move.
   * 
   * @param speed the speed to move the motors at
   * @return the command
   */
  public Command moveClimb(Double speed) {
    return run(()->m_RightSparkMax.set(speed));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

