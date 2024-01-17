// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsysem;
import frc.robot.subsystems.SwerveSubsystem;

public class ShooterSetSpeedInterpolation extends Command {
  private final ShooterSubsysem shooterSubsystem = ShooterSubsysem.getInstance();
  private final SwerveSubsystem swerve = SwerveSubsystem.getInstance();

  /** Creates a new ShooterSetSpeedInterpolation. */
  public ShooterSetSpeedInterpolation() {
    this.addRequirements(shooterSubsystem, swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.shooterSubsystem.setShooterSpeed(shooterSubsystem.InterpolationValue(swerve.getPose()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
