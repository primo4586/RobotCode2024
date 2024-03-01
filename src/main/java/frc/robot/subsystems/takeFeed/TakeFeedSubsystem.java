// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.takeFeed;

import static frc.robot.Constants.CAN_BUS_NAME;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.takeFeed.TakeFeedConstants.takeFeedConstants;

public class TakeFeedSubsystem extends SubsystemBase {
  
  private TalonFX m_motor;
  private DigitalInput m_opticSensor;

  private VoltageOut voltageOut = new VoltageOut(0, false, true, false, false);

  private TakeFeedSubsystem() {
    m_motor = new TalonFX(takeFeedConstants.MOTOR_ID, CAN_BUS_NAME);


    TalonFXConfiguration motorConfig = new TalonFXConfiguration();

    motorConfig.CurrentLimits.SupplyCurrentLimit = 30;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfig.CurrentLimits.SupplyCurrentThreshold = 60;
    motorConfig.CurrentLimits.SupplyTimeThreshold = 0.1;
    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    m_motor.getConfigurator().apply(motorConfig);
  }

  public void setSpeed(double voltage) {
    m_motor.setControl(voltageOut.withOutput(voltage));
  }

  public boolean getOpticSensorValue() {
    return m_opticSensor.get();
  }

  public double getSpeed() {
    return m_motor.getVelocity().getValueAsDouble();
  }

  public boolean atShootSpeed(){
    return getSpeed() >= takeFeedConstants.AT_SHOOTING_SPEED;
  }

  @Override
  public void periodic() {
  }
  
  // singelton
  private static TakeFeedSubsystem instance;

  public static TakeFeedSubsystem getInstance() {
    if (instance == null) {
      instance = new TakeFeedSubsystem();
    }
    return instance;
  }
}
