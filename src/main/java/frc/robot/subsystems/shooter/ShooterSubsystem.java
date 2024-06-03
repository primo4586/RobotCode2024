// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Misc;

//TODO: add sysid
/**
 * Shooter subsystem, controls two TalonFX motors to spin the shooter wheels
 */
public class ShooterSubsystem extends SubsystemBase implements ShooterConstants {
  private final TalonFX m_UMotor = new TalonFX(UP_MOTOR_SHOOTER_ID, Misc.CAN_BUS_NAME);
  private final TalonFX m_DMotor = new TalonFX(DOWN_MOTOR_SHOOTER_ID, Misc.CAN_BUS_NAME);
  private final MotionMagicVelocityTorqueCurrentFOC m_mmReqest = new MotionMagicVelocityTorqueCurrentFOC(0);

  private static ShooterSubsystem INSTANCE;

  /**
   * Returns the instance of this subsystem
   * 
   * @return The instance of this subsystem
   */
  public static ShooterSubsystem getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new ShooterSubsystem();
    }
    return INSTANCE;
  }

  /**
   * Creates a new ShooterSubsystem. This class is a singleton so it should only
   * be
   * constructed once
   */
  private ShooterSubsystem() {
    applyMotorsConfig();
  }

  /**
   * Sets the velocity of the shooter motors
   * 
   * @param velocity The velocity to set the shooter motors to
   * @return A command to set the velocity of the shooter motors
   */
  public Command setShooterVel(double velocity) {
    return this.runOnce(() -> {
      m_UMotor.setControl(m_mmReqest.withVelocity(velocity));
      m_DMotor.setControl(m_mmReqest.withVelocity(velocity));
    });

  }

  /**
   * Sets the velocity of the shooter motors to the default speekr velocity
   * 
   * @return A command to set the velocity of the shooter motors to the default
   *         speekr velocity
   */
  public Command setSpeakerVel() {
    return runEnd(() -> setShooterVel(SPEAKR_VELOCITY), () -> setShooterVel(IDLE_VELOCITY));
  }

  /**
   * Checks if the shooter motors are at the requested velocity
   * 
   * @return True if the shooter motors are at the requested velocity
   */
  public boolean isMotorsAtVel() {
    return Math.abs(m_UMotor.getVelocity().getValue() - m_mmReqest.Velocity) < MAX_VEL_ERROR
        && Math.abs(m_DMotor.getVelocity().getValue() - m_mmReqest.Velocity) < MAX_VEL_ERROR;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Sets the signal update frequency for the shooter motors to 1000 Hz for
   * characterization
   */
  @SuppressWarnings("unused")
  private void sysidHigherSignalRate() {

    /* Speed up signals for better characterization data */
    BaseStatusSignal.setUpdateFrequencyForAll(1000, m_UMotor.getVelocity());
    BaseStatusSignal.setUpdateFrequencyForAll(1000, m_DMotor.getVelocity());

    m_UMotor.optimizeBusUtilization();
    m_DMotor.optimizeBusUtilization();
  }

  /**
   * Configures the shooter motors with the PID constants and other settings
   */
  private void applyMotorsConfig() {
    TalonFXConfiguration upCfg = new TalonFXConfiguration();

    Slot0Configs upSlot0 = upCfg.Slot0;
    upSlot0.kP = UP_KP;
    upSlot0.kD = UP_KD;
    upSlot0.kS = UP_KS;
    upSlot0.kA = UP_KA;

    MotionMagicConfigs upMM = upCfg.MotionMagic;
    upMM.MotionMagicAcceleration = MOTION_MAGIC_ACCELERATION;
    upMM.MotionMagicJerk = MOTION_MAGIC_JERK;

    FeedbackConfigs upFdb = upCfg.Feedback;
    upFdb.SensorToMechanismRatio = SENSOR_TO_MEC_RATIO;

    MotorOutputConfigs upMotorOutputConfigs = upCfg.MotorOutput;
    upMotorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;// TODO: constants
    upMotorOutputConfigs.NeutralMode = NeutralModeValue.Coast;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = m_UMotor.getConfigurator().apply(upCfg);
      if (status.isOK())
        break;
    }
    if (!status.isOK()) {
      System.out.println("Could not configure up motor. Error: " + status.toString());
    }

    TalonFXConfiguration downCfg = new TalonFXConfiguration();

    Slot0Configs downSlot0 = downCfg.Slot0;
    downSlot0.kP = DOWN_KP;
    downSlot0.kD = DOWN_KD;
    downSlot0.kS = DOWN_KS;
    downSlot0.kA = DOWN_KA;

    MotionMagicConfigs downMM = downCfg.MotionMagic;
    downMM.MotionMagicAcceleration = MOTION_MAGIC_ACCELERATION;
    downMM.MotionMagicJerk = MOTION_MAGIC_JERK;

    FeedbackConfigs downFdb = downCfg.Feedback;
    downFdb.SensorToMechanismRatio = SENSOR_TO_MEC_RATIO;

    MotorOutputConfigs downMotorOutputConfigs = downCfg.MotorOutput;
    downMotorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
    downMotorOutputConfigs.NeutralMode = NeutralModeValue.Coast;

    status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = m_DMotor.getConfigurator().apply(downCfg);
      if (status.isOK())
        break;
    }
    if (!status.isOK()) {
      System.out.println("Could not configure down motor. Error: " + status.toString());
    }
  }
}
