// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.basicCommands.ShooterArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterArmSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class ShooterArmSpeakerAngle extends Command {
  private final ShooterArmSubsystem shooterArmSubsystem = ShooterArmSubsystem.getInstance();
  private final SwerveSubsystem swerve = SwerveSubsystem.getInstance();

  /** Creates a new ShooterAngleFromDistanceInterpolation. */
  public ShooterArmSpeakerAngle() {
    this.addRequirements(shooterArmSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterArmSubsystem.moveArmTo(shooterArmSubsystem.angleFromDistance(swerve.getPose()));
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
