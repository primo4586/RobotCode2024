// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.basicCommands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsysem;
import frc.robot.subsystems.SwerveSubsystem;

public class ShooterSpeakerSetSpeed extends Command {
  private final ShooterSubsysem shooterSubsystem = ShooterSubsysem.getInstance();
  private final SwerveSubsystem swerve = SwerveSubsystem.getInstance();

  /** Creates a new ShooterSetSpeedInterpolation. */
  public ShooterSpeakerSetSpeed() {
    this.addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.shooterSubsystem.setShooterSpeed(shooterSubsystem.speakerInterpolate(swerve.getPose()));
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
