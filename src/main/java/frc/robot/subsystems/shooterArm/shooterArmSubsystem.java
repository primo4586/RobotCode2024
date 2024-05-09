// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooterArm;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.MiscConstants;

import static frc.robot.subsystems.shooterArm.shooterArmConstants.*;

//TODO: add manual home
//TODO: add speakerInterpolate
//TODO: add sysid
public class shooterArmSubsystem extends SubsystemBase {

  private final TalonFX m_Motor = new TalonFX(MOTOR_ID, MiscConstants.CAN_BUS_NAME);
  private final MotionMagicExpoTorqueCurrentFOC mm = new MotionMagicExpoTorqueCurrentFOC(0);

  private static shooterArmSubsystem INSTANCE;

  public static shooterArmSubsystem getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new shooterArmSubsystem();
    }
    return INSTANCE;
  }

  private shooterArmSubsystem() {
    applyMotorsConfig();
  }

  /**
   * @param distance distance from speaker
   */
  public Command speakerAngle(double distance) {
    return moveArmTO(SPEAKER_ANGLE_EXTERPOLATION.exterpolate(distance));
  }

  /**
   * Moves the shooter arm to a specific position. Will only run once.
   * 
   * @param position The position to move the arm to
   */
  public Command moveArmTO(double position) {
    return runOnce(
        // Set the control mode to position
        // And set the target position to the one passed in
        () -> m_Motor.setControl(mm.withPosition(position)));

  }

  /**
   * homes the arms and zero the encoder
   */
  public Command homeArm() {
    return prepareHome().andThen(
        runEnd(() -> {
          if (!getReverseLimit()) {
            m_Motor.set(shooterArmConstants.RESET_SPEED);
          }
        }, () -> m_Motor.set(0)).withTimeout(10));
  }

  public Command prepareHome() {
    return getReverseLimit()
        ? Commands.none()
        : (runOnce(() -> m_Motor.set(-RESET_SPEED * 3)).andThen(Commands.waitUntil(() -> !getReverseLimit())))
            .withTimeout(3);
  }

  public double getArmPose() {
    return m_Motor.getPosition().getValue();
  }

  // Checking the degree difference conditions
  public boolean isArmReady() {
    return (Math.abs(getArmPose() - mm.Position) < shooterArmConstants.MINIMUM_ERROR);
  }

  public void coast() {
    m_Motor.setNeutralMode(NeutralModeValue.Coast);
  }

  public void breakMode() {
    m_Motor.setNeutralMode(NeutralModeValue.Brake);
  }

  public boolean getReverseLimit() {
    return m_Motor.getReverseLimit().getValue() == lIMIT_SWITCH_TRUE_VALUE;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private void applyMotorsConfig() {
    TalonFXConfiguration shooterAngleCfg = new TalonFXConfiguration();

    Slot0Configs slot0Configs = shooterAngleCfg.Slot0;
    slot0Configs.kP = KP;
    slot0Configs.kD = KD;
    slot0Configs.kS = KS;
    slot0Configs.kA = KA;

    MotionMagicConfigs motionMagicConfigs = shooterAngleCfg.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = MM_CRUISE;
    motionMagicConfigs.MotionMagicAcceleration = MM_ACCELERATION;
    motionMagicConfigs.MotionMagicJerk = MM_JERK;

    FeedbackConfigs feedbackConfigs = shooterAngleCfg.Feedback;
    feedbackConfigs.SensorToMechanismRatio = SENSOR_TO_MEC_RATIO;

    MotorOutputConfigs motorOutputConfigs = shooterAngleCfg.MotorOutput;
    motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;// TODO: constants
    motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;

    HardwareLimitSwitchConfigs hardwareLimitSwitchConfigs = shooterAngleCfg.HardwareLimitSwitch;
    hardwareLimitSwitchConfigs.ReverseLimitAutosetPositionEnable = true;
    hardwareLimitSwitchConfigs.ReverseLimitAutosetPositionValue = RESET_POSE;
    hardwareLimitSwitchConfigs.ReverseLimitEnable = true;

    SoftwareLimitSwitchConfigs softwareLimitSwitchConfigs = shooterAngleCfg.SoftwareLimitSwitch;
    softwareLimitSwitchConfigs.ForwardSoftLimitEnable = true;
    softwareLimitSwitchConfigs.ForwardSoftLimitThreshold = FORWARD_LIMIT;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = m_Motor.getConfigurator().apply(shooterAngleCfg);
      if (status.isOK())
        break;
    }
    if (!status.isOK()) {
      System.out.println("Could not configure shooter Angle motor. Error: " + status.toString());
    }
  }
}
