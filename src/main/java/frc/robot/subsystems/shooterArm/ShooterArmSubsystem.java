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
//TODO: add sysid
public class ShooterArmSubsystem extends SubsystemBase {

  private final TalonFX m_Motor = new TalonFX(MOTOR_ID, MiscConstants.CAN_BUS_NAME);
  private final MotionMagicExpoTorqueCurrentFOC mm = new MotionMagicExpoTorqueCurrentFOC(0);

  private static ShooterArmSubsystem INSTANCE;

  /**
   * Get an instance of the shooter arm subsystem
   * 
   * @return The instance of the shooter arm subsystem
   */
  public static ShooterArmSubsystem getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new ShooterArmSubsystem();
    }
    return INSTANCE;
  }

  /**
   * Constructor that sets the motors config
   */
  private ShooterArmSubsystem() {
    applyMotorsConfig();
  }

  /**
   * Create a command that will move the shooter arm to a specific angle based on
   * the distance from the speaker
   * 
   * @param distance The distance from the speaker
   * @return The command
   */
  public Command speakerAngleEterapolateCommand(double distance) {
    return moveArmToCommand(SPEAKER_ANGLE_EXTERPOLATION.exterpolate(distance));
  }

  /**
   * Create a command that will move the shooter arm to a specific position. Will
   * only run once.
   * 
   * @param position The position to move the arm to
   * @return The command
   */
  public Command moveArmToCommand(double position) {
    return runOnce(
        // Set the control mode to position
        // And set the target position to the one passed in
        () -> m_Motor.setControl(mm.withPosition(position)));
  }

  /**
   * Create a command that will home the shooter arm and set the encoder to 0.
   * 
   * @return The command
   */
  public Command homeArmcCommand() {
    return prepareHomeCommand().andThen(
        runEnd(() -> {
          if (!getReverseLimit()) {
            m_Motor.set(shooterArmConstants.RESET_SPEED);
          }
        }, () -> m_Motor.set(0)).withTimeout(10));
  }

  /**
   * Prepare the home command, if the reverse limit switch is pressed, do
   * nothing, otherwise move the motor to the reverse limit switch position
   * at a high speed and wait for the switch to be pressed
   * 
   * @return The command
   */
  private Command prepareHomeCommand() {

    return !getReverseLimit()
        ? Commands.none()
        : (runOnce(() -> m_Motor.set(-RESET_SPEED * 3)).andThen(Commands.waitUntil(() -> !getReverseLimit())))
            .withTimeout(3);
  }

  /**
   * Get the current position of the shooter arm
   * 
   * @return The position of the shooter arm
   */
  public double getArmPose() {
    return m_Motor.getPosition().getValue();
  }

  /**
   * Check if the shooter arm is ready to fire based on the degree difference
   * 
   * @return True if the shooter arm is ready
   */
  public boolean isArmReady() {
    return (Math.abs(getArmPose() - mm.Position) < shooterArmConstants.MINIMUM_ERROR);
  }

  /**
   * Set the motor to coast mode
   */
  public void motorCoast() {
    m_Motor.setNeutralMode(NeutralModeValue.Coast);
  }

  /**
   * Set the motor to brake mode
   */
  public void motorBreak() {
    m_Motor.setNeutralMode(NeutralModeValue.Brake);
  }

  /**
   * Get the state of the reverse limit switch
   * 
   * @return The state of the reverse limit switch
   */
  public boolean getReverseLimit() {
    return m_Motor.getReverseLimit().getValue() == lIMIT_SWITCH_TRUE_VALUE;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Apply the motors config
   */
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
