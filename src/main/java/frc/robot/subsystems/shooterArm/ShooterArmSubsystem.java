// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooterArm;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.shooterArm.ShooterArmConstants.shooterArmConstants;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.util.AllianceFlipUtil;
import frc.util.interpolation.InterpolateUtil;

public class ShooterArmSubsystem extends SubsystemBase {

  private final SwerveSubsystem swerve = SwerveSubsystem.getInstance();
  private TalonFX m_shooterArmMotor;
  DigitalInput limitSwitch = new DigitalInput(shooterArmConstants.SWITCH_ID);
  private final MotionMagicVoltage motionMagic = new MotionMagicVoltage(0,
      false,
      0.0,
      0,
      true,
      false,
      false);

  double targetPose = 0;

  private ShooterArmSubsystem() {

    this.m_shooterArmMotor = new TalonFX(shooterArmConstants.SHOOTER_ARM_ID, Constants.CAN_BUS_NAME);

    // create the full MotionMagic
    TalonFXConfiguration configuration = new TalonFXConfiguration();
    MotionMagicConfigs mm = new MotionMagicConfigs();

    mm.MotionMagicCruiseVelocity = shooterArmConstants.MM_CRUISE;
    mm.MotionMagicAcceleration = shooterArmConstants.MM_ACCELERATION;
    mm.MotionMagicJerk = shooterArmConstants.MM_JERK;
    configuration.MotionMagic = mm;

    configuration.Slot0.kP = shooterArmConstants.KP;
    configuration.Slot0.kD = shooterArmConstants.KD;
    configuration.Slot0.kV = shooterArmConstants.KV;
    configuration.Slot0.kS = shooterArmConstants.KS;

    configuration.Voltage.PeakForwardVoltage = shooterArmConstants.PEEK_FORWARD_VOLTAGE;
    configuration.Voltage.PeakReverseVoltage = shooterArmConstants.PEEK_REVERSE_VOLTAGE;

    // forward and backward limits
    configuration.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    configuration.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    configuration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = shooterArmConstants.FORWARD_LIMIT;
    configuration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = shooterArmConstants.BACKWARD_LIMIT;

    configuration.Feedback.SensorToMechanismRatio = shooterArmConstants.TICKS_PER_DEGREE;

    configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    StatusCode statusCode = StatusCode.StatusCodeNotInitialized;

    for (int i = 0; i < 5; i++) {
      statusCode = m_shooterArmMotor.getConfigurator().apply(configuration);
      if (statusCode.isOK())
        break;
    }
    if (!statusCode.isOK())
      System.out.println("Shooter Arm could not apply config, error code:" + statusCode.toString());

    m_shooterArmMotor.setPosition(shooterArmConstants.SHOOTER_ARM_START_POSE);

  }

  // set the position of the arm
  public void setPosition(double pose) {
    m_shooterArmMotor.setPosition(pose);
  }

  // moving function for the Arm
  public void moveArmTo(double degrees) {
    degrees = degrees<0?0:degrees;
    degrees = degrees>80?80:degrees;
    targetPose = degrees;
    m_shooterArmMotor.setControl(motionMagic.withPosition(degrees));
  }

  // get function for the Arm pose
  public double getArmPose() {
    return m_shooterArmMotor.getPosition().getValue();
  }

  // Checking the degree difference conditions
  public boolean isArmReady() {
    if(RobotState.isAutonomous()){
      return (Math.abs(getArmPose() - targetPose) < 2);
    }
    return (Math.abs(getArmPose() - targetPose) < shooterArmConstants.MINIMUM_ERROR);
  }

  // geting if the switch is open
  public boolean getSwitch() {
    return !this.limitSwitch.get();
  }

  public void setSpeed(double speed) {
    m_shooterArmMotor.set(speed);
  }

  public void coast() {
    m_shooterArmMotor.setNeutralMode(NeutralModeValue.Coast);
  }

  public void breakMode() {
    m_shooterArmMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public void manualZeroShooterArm() {
    coast();
    if (getSwitch()) {
      while (getSwitch()) {
      }
    }
    while (!getSwitch()) {
    }
    SmartDashboard.putBoolean("zerod out shooter", true);
    setPosition(0);
    breakMode();
  }

  public double speakerInterpolate() {
    double yOffset = Math.abs(swerve.getPose().getY()-FieldConstants.Speaker.centerSpeakerOpening.getY());
    return InterpolateUtil.interpolate(
        shooterArmConstants.SHOOTER_ANGLE_INTERPOLATION_MAP,
        swerve.getPose().getTranslation().getDistance(
            AllianceFlipUtil.apply(FieldConstants.Speaker.centerSpeakerOpening).getTranslation()))
            - (yOffset>0.5?yOffset*1:0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("shooter arm switch", getSwitch());
  }

  private static ShooterArmSubsystem instance;

  public static ShooterArmSubsystem getInstance() {
    if (instance == null) {
      instance = new ShooterArmSubsystem();
    }
    return instance;
  }
}
