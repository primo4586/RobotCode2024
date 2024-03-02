// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.shooterArm.ShooterArmConstants;
import frc.robot.subsystems.shooterArm.ShooterArmSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.util.interpolation.InterpolationMap;
import frc.util.shuffleboardAlike.AutoContainer;
import frc.util.shuffleboardAlike.PrimoShuffleboard;
import frc.util.swerve.CTREConfigs;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static final CTREConfigs ctreConfigs = new CTREConfigs();

  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  
  private AutoContainer autoContainer;
  double offset = -0.6;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    autoContainer = new AutoContainer();
    PrimoShuffleboard.getInstance().initDashboard();
    enableLiveWindowInTest(true);
    SmartDashboard.putNumber("offset", offset);

    ShooterArmConstants.shooterArmConstants.SHOOTER_ANGLE_INTERPOLATION_MAP = new InterpolationMap()
      .put(2.13 + offset, 20)
      .put(3.2 + offset, 37)
      .put(3.655 + offset, 40.5)
      .put(3.29 + offset, 39)
      .put(5.441276465250574 + offset, 52.8)
      .put(2.135 + offset, 20)
      .put(2.19 + offset, 18)
      .put(1.995 + offset, 15)
      .put(1.46 + offset, 9)
      .put(1 + offset, 0);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    if(SmartDashboard.getNumber("offset", offset) != offset){
      offset = SmartDashboard.getNumber("offset", offset);
      ShooterArmConstants.shooterArmConstants.SHOOTER_ANGLE_INTERPOLATION_MAP = new InterpolationMap()
      .put(2.13 + offset, 20)
      .put(3.2 + offset, 37)
      .put(3.655 + offset, 40.5)
      .put(3.29 + offset, 39)
      .put(5.441276465250574 + offset, 52.8)
      .put(2.135 + offset, 20)
      .put(2.19 + offset, 18)
      .put(1.995 + offset, 15)
      .put(1.46 + offset, 9)
      .put(1 + offset, 0);
    }
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    if(RobotController.getUserButton()){
      SwerveSubsystem.getInstance().zeroGyro();
      ShooterArmSubsystem.getInstance().manualZeroShooterArm();
    }
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = autoContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
