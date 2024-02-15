// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

import static frc.robot.Constants.IntakeArmConstants.*;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

public class IntakeArmSubsystem extends SubsystemBase {
  private TalonFX m_IntakeArmMotor;
  private final MotionMagicVoltage motionMagic = new MotionMagicVoltage(intakeArmStartingValue);
  private final DigitalInput intakeArmSwitch = new DigitalInput(intakeArmSwitchID);

  // singleton
  private static IntakeArmSubsystem instance;

  public static IntakeArmSubsystem getInstance() {
    if (instance == null) {
      instance = new IntakeArmSubsystem();
    }
    return instance;
  }

  /** Creates a new IntakeArmSubsystem. */
  private IntakeArmSubsystem() {
    this.m_IntakeArmMotor = new TalonFX(IntakeArmMotorID, Constants.canBus_name);
    TalonFXConfiguration configs = new TalonFXConfiguration();
    MotionMagicConfigs mm = new MotionMagicConfigs();

    mm.MotionMagicCruiseVelocity = mmVelocity;
    mm.MotionMagicAcceleration = mmAcceleration;
    mm.MotionMagicJerk = mmJerk;
    configs.MotionMagic = mm;

    configs.Slot0.kP = KP;
    configs.Slot0.kD = KD;
    configs.Slot0.kV = KV;
    configs.Slot0.kS = KS;

    configs.Voltage.PeakForwardVoltage = PeakForwardVoltage;
    configs.Voltage.PeakReverseVoltage = PeakReverseVoltage;
    configs.Feedback.SensorToMechanismRatio = TICKS_PER_DEGREE;

    configs.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    configs.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

    // configs.SoftwareLimitSwitch.ForwardSoftLimitEnable = ForwardSoftLimitEnable;
    // configs.SoftwareLimitSwitch.ReverseSoftLimitEnable = ReverseSoftLimitEnable;
    configs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ForwardSoftLimitThreshold;
    configs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = RevesrseSoftLimitThreshold;

    configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // gives code to TalonFX
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; i++) {
      status = m_IntakeArmMotor.getConfigurator().apply(configs);
      if (status.isOK()) {
        break;
      }
    }
    if (!status.isOK()) {
      System.out.println("intake arm could not apply configs, error code: " + status.toString());
    }

    m_IntakeArmMotor.setPosition(intakeArmStartingValue);
  }

  // moving the arm
  public void moveArmTo(double degrees) {
    m_IntakeArmMotor.setControl(motionMagic.withPosition(degrees));
  }

  // set speed
  public void setSpeed(DoubleSupplier speed) {
    m_IntakeArmMotor.set(speed.getAsDouble());
  }

  // set encoder
  public void setEncoder(double encoderValue) {
    m_IntakeArmMotor.setPosition(encoderValue);
  }

  // get if a switch is press
  public boolean getSwitch() {
    return !intakeArmSwitch.get();
  }

  // check if intake arm is in place
  public boolean checkIntakeArmPosion() {
    return Math.abs(m_IntakeArmMotor.getClosedLoopError().getValue()) < minimumError;
  }

  public void runCharacterizationVolts(Double voltage) {
    m_IntakeArmMotor.setVoltage(voltage);
  }

  public double getCharacterizationVelocity() {
    return m_IntakeArmMotor.getVelocity().getValueAsDouble();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putBoolean("intkeArmSwitch", getSwitch());
    SmartDashboard.putNumber("intakeArm pose", m_IntakeArmMotor.getPosition().getValueAsDouble());
  }


  
    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutableMeasure<Angle> m_angle = mutable(Rotations.of(0));
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutableMeasure<Velocity<Angle>> m_velocity = mutable(RotationsPerSecond.of(0));

    // Create a new SysId routine for characterizing the shooter.
  private final SysIdRoutine m_sysIdRoutine =
      new SysIdRoutine(
          // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
          new SysIdRoutine.Config(Volts.of(1).per(Seconds.of(1)), Volts.of(10), Seconds.of(10)),
          new SysIdRoutine.Mechanism(
              // Tell SysId how to plumb the driving voltage to the motor(s).
              (Measure<Voltage> volts) -> {
                m_IntakeArmMotor.setVoltage(volts.in(Volts));
              },
              // Tell SysId how to record a frame of data for each motor on the mechanism being
              // characterized.
              log -> {
                // Record a frame for the shooter motor.
                log.motor("shooter-wheel")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            m_IntakeArmMotor.getSupplyVoltage().getValueAsDouble(), Volts))
                    .angularPosition(m_angle.mut_replace(m_IntakeArmMotor.getPosition().getValueAsDouble(), Rotations))
                    .angularVelocity(
                        m_velocity.mut_replace(m_IntakeArmMotor.getVelocity().getValueAsDouble(), RotationsPerSecond));
              },
              // Tell SysId to make generated commands require this subsystem, suffix test state in
              // WPILog with this subsystem's name ("shooter")
              this));

                /**
   * Returns a command that will execute a quasistatic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdQuasistatic() {
    return m_sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward);
  }

  /**
   * Returns a command that will execute a dynamic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdDynamic() {
    return m_sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward);
  }
}
