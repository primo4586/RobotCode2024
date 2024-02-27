// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.basicCommands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.basicCommands.SwerveCommands.FieldConstants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class ShooterSpeaker extends Command {
  private final ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();
  private final SwerveSubsystem swerveSubsystem = SwerveSubsystem.getInstance();

  /** Creates a new ShooterSetSpeedInterpolation. */
  public ShooterSpeaker() {
    this.addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double distance = SwerveSubsystem.getInstance().getPose().getTranslation().getDistance(FieldConstants.Speaker.centerSpeakerOpening.getTranslation());
    double speed = 70;
    if(distance>2.6){
      speed = 100;
    }
    if(distance<2.6){
      speed = 70;
    }
    this.shooterSubsystem.setShooterSpeed(speed);
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
