// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.MiscConstants;

import static frc.robot.subsystems.intake.intakeConstants.*;

public class IntakeSubsystem extends SubsystemBase {
  private TalonFX m_motor = new TalonFX(MOTOR_ID, MiscConstants.CAN_BUS_NAME);
  private DigitalInput m_opticSensor = new DigitalInput(OPTIC_SENSOR_ID);
  private TorqueCurrentFOC currentFOC = new TorqueCurrentFOC(0);

  private static IntakeSubsystem INSTANCE;

  public static IntakeSubsystem getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new IntakeSubsystem();
    }
    return INSTANCE;
  }

  private IntakeSubsystem() {
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();

    motorConfig.CurrentLimits.SupplyCurrentLimit = 40;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfig.CurrentLimits.SupplyCurrentThreshold = 100;
    motorConfig.CurrentLimits.SupplyTimeThreshold = 0.2;
    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    m_motor.getConfigurator().apply(motorConfig);
  }

  public Command setCurrent(double current) {
    return this.runOnce(() -> m_motor.setControl(currentFOC.withOutput(current)));
  }
  
  public Command intakeCommand() {
    return this.runEnd(()->setCurrent(INTAKE_CURRENT), ()->m_motor.stopMotor()).until(()->getOpticSensorValue());
  }
  
  public Command feedShooterCommand() {
    return this.runEnd(() -> setCurrent(FEED_SHOOTER_CURRENT), () -> m_motor.stopMotor())
        .withTimeout(FEED_SHOOTER_TIME);
  }
  
  public Command setSpeedCommand(double speed) {
    return this.runOnce(()->m_motor.set(speed));
  }

  public boolean getOpticSensorValue() {
    return m_opticSensor.get();
  }

  @Override
  public void periodic() {
  }
}
