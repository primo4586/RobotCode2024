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

import edu.wpi.first.math.geometry.Pose2d;
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
import frc.Utils.interpolation.InterpolateUtil;
import frc.Utils.vision.Vision;
import frc.robot.Constants;

import static frc.robot.Constants.ShooterArmConstants.*;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

public class ShooterArmSubsystem extends SubsystemBase {
  /** Creates a new ShooterArmSubsystem. */

  // created the motor and MotionMagic
  private TalonFX m_shooterArmMotor;
  private final MotionMagicVoltage motionMagic = new MotionMagicVoltage(shooterArmStartPose);
  DigitalInput limitSwitch;
  private final Vision vision = Vision.getInstance();

  // the instance
  private static ShooterArmSubsystem instance;

  public static ShooterArmSubsystem getInstance() {
    if (instance == null) {
      instance = new ShooterArmSubsystem();
    }
    return instance;
  }

  private ShooterArmSubsystem() {
    this.m_shooterArmMotor = new TalonFX(ShooterArmID, Constants.canBus_name);
    this.limitSwitch = new DigitalInput(SwitchID);

    // create the full MotionMagic
    TalonFXConfiguration configuration = new TalonFXConfiguration();
    MotionMagicConfigs mm = new MotionMagicConfigs();

    mm.MotionMagicCruiseVelocity = mmCruise;
    mm.MotionMagicAcceleration = mmAcceleration;
    mm.MotionMagicJerk = mmJerk;
    configuration.MotionMagic = mm;

    configuration.Slot0.kP = kp;
    configuration.Slot0.kD = kd;
    configuration.Slot0.kV = kv;
    configuration.Slot0.kS = ks;

    configuration.Voltage.PeakForwardVoltage = peekForwardVoltage;
    configuration.Voltage.PeakReverseVoltage = peekReverseVoltage;

    // forward and backword limits
    configuration.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;//TODO: change to true
    configuration.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    configuration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = forwardLimit;
    configuration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = backwordLimit;

    configuration.Feedback.SensorToMechanismRatio = TICKS_PER_DEGREE;
    
    configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    StatusCode statusCode = StatusCode.StatusCodeNotInitialized;

    for (int i = 0; i < 5; i++) {
      statusCode = m_shooterArmMotor.getConfigurator().apply(configuration);
      if (statusCode.isOK())
        break;
    }
    if (!statusCode.isOK())
      System.out.println("shooter Arm could not apply config, error code:" + statusCode.toString());

    m_shooterArmMotor.setPosition(shooterArmStartPose);

  }

  // set the position of the arm
  public void setPosition(double pose) {
    m_shooterArmMotor.setPosition(pose);
  }

  // moving function for the Arm
  public void moveArmTo(double degrees) {
    m_shooterArmMotor.setControl(motionMagic.withPosition(degrees));
  }

  // get function for the Arm pose
  public double getArmPose() {
    return m_shooterArmMotor.getPosition().getValue();
  }

  // Checking the degree difference conditions
  public boolean isArmReady() {
    return (Math.abs(m_shooterArmMotor.getClosedLoopError().getValue()) < minimumError);
  }

  // geting if the switch is open
  public boolean getSwitch() {
    return !this.limitSwitch.get();
  }

  public void setSpeedArm(DoubleSupplier speed) {
    m_shooterArmMotor.set(speed.getAsDouble());
  }

  public double angleFromDistance(Pose2d pose) {
    return InterpolateUtil.interpolate(SHOOTER_ANGLE_INTERPOLATION_MAP, vision.DistanceFromTarget(pose));
  }

  public void runCharacterizationVolts(Double voltage) {
    m_shooterArmMotor.setVoltage(voltage);
  }

  public double getCharacterizationVelocity() {
    return m_shooterArmMotor.getVelocity().getValueAsDouble();
  }

  @Override
  public void periodic() {

    SmartDashboard.putBoolean("ShooterArmSwitch", getSwitch());
    SmartDashboard.putNumber("ShooterArm pose", getArmPose());
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
                m_shooterArmMotor.setVoltage(volts.in(Volts));
              },
              // Tell SysId how to record a frame of data for each motor on the mechanism being
              // characterized.
              log -> {
                // Record a frame for the shooter motor.
                log.motor("shooter-wheel")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            m_shooterArmMotor.getSupplyVoltage().getValueAsDouble(), Volts))
                    .angularPosition(m_angle.mut_replace(m_shooterArmMotor.getPosition().getValueAsDouble(), Rotations))
                    .angularVelocity(
                        m_velocity.mut_replace(m_shooterArmMotor.getVelocity().getValueAsDouble(), RotationsPerSecond));
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
