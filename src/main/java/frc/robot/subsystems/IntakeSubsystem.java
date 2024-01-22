// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.IntakeConstants.*;

public class IntakeSubsystem extends SubsystemBase {
  private TalonSRX m_Intake;
  DigitalInput lazerSensor;

  private static IntakeSubsystem instance;
  
    public static IntakeSubsystem getInstance() {
      if (instance == null) {
        instance = new IntakeSubsystem();
      }
      return instance;
    }

  /** Creates a new IntakeSubsystem. */
  private IntakeSubsystem() {
    this.m_Intake = new TalonSRX(IntakeMotorID);
    this.lazerSensor = new DigitalInput(SwitchID);
  }

  public void setSpeed(double speed){
    m_Intake.set(ControlMode.PercentOutput, speed);
  }

  public boolean getSwitch(){
    return lazerSensor.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
