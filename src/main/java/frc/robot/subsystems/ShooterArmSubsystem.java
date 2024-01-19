// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ShooterArmConstants.*;

public class ShooterArmSubsystem extends SubsystemBase {
  /** Creates a new ShooterArmSubsystem. */

    // created the motor and MotionMagic
    private TalonFX m_ArmMotor;
    private final MotionMagicVoltage motionMagic = new MotionMagicVoltage(0);
    DigitalInput limitSwitch; 
  
    // the instance
    private static ShooterArmSubsystem instance;
  
    public static ShooterArmSubsystem getInstance() {
      if (instance == null) {
        instance = new ShooterArmSubsystem();
      }
      return instance;
    }

  private ShooterArmSubsystem() {
    this.m_ArmMotor = new TalonFX(ShooterArmID);
    this.limitSwitch = new DigitalInput(SwitchID);

    // creat the full MotionMagic
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
    configuration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    configuration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = forwardLimit;
    configuration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    configuration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = backwordLimit;

    // set Ratio to 50:1
    configuration.Feedback.SensorToMechanismRatio = TICKS_PER_DEGREE;

    StatusCode statusCode = StatusCode.StatusCodeNotInitialized;

    for (int i = 0; i < 5; i++) {
      statusCode = m_ArmMotor.getConfigurator().apply(configuration);
      if (statusCode.isOK())
        break;
    }
    if (!statusCode.isOK())
      System.out.println("Arm could not apply config, error code:" + statusCode.toString());

    m_ArmMotor.setPosition(0);

  }

  //set the postition of the arm
  public void setPosition(double pose){
    m_ArmMotor.setPosition(pose);
  }

  // moving function for the Arm
  public void moveArmTo(double degrees) {
    m_ArmMotor.setControl(motionMagic.withPosition(degrees));
  }

  // get function for the Arm pose
  public double getArmPose() {
    return m_ArmMotor.getPosition().getValue();
  }

  // Checking the degree difference conditions
  public boolean isArmReady() {
    return (Math.abs(m_ArmMotor.getClosedLoopError().getValue()) < minimumError);
  }

  //geting if the switch is open
  public boolean getSwitch(){
    return this.limitSwitch.get();
  }

  public void movementArmToReset(){
    m_ArmMotor.set(resetSpeed);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
