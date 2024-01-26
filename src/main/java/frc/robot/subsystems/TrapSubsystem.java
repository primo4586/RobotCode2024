// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import static frc.robot.Constants.trapConstants.*;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TrapSubsystem extends SubsystemBase {
  private WPI_TalonSRX m_trapMotor;
  private DigitalInput m_trapNoteSensor;

  private static TrapSubsystem instance;

  public static TrapSubsystem getInstance() {
    if (instance == null) {
      instance = new TrapSubsystem();
    }
    return instance;
  }

  /** Creates a new SubsystemTrap. */
  private TrapSubsystem() {
    m_trapMotor = new WPI_TalonSRX(TRAP_MOTOR_ID);
    m_trapNoteSensor = new DigitalInput(TrapNoteSensorID);
  }

  public void setSpeed(double speed) {
    m_trapMotor.set(speed);
  }

  public boolean isNoteDetected() {
    return m_trapNoteSensor.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
