// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.IntakeConstants.*;

public class IntakeSubsystem extends SubsystemBase {
  private CANSparkMax m_Intake;
  DigitalInput m_intakeNoteSensor;

  private static IntakeSubsystem instance;

  public static IntakeSubsystem getInstance() {
    if (instance == null) {
      instance = new IntakeSubsystem();
    }
    return instance;
  }

  /** Creates a new IntakeSubsystem. */
  private IntakeSubsystem() {
    this.m_Intake = new CANSparkMax(IntakeMotorID, MotorType.kBrushless);
    this.m_intakeNoteSensor = new DigitalInput(intakeNoteSensorID);


  }

  public void setSpeed(double speed) {
    m_Intake.set(speed);
  }

  public boolean getSwitch() {
    return m_intakeNoteSensor.get();
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("sensor", getSwitch());
    // This method will be called once per scheduler run
  }
}
