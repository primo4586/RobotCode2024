// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.Utils.vision.Vision;
import frc.robot.subsystems.ShooterArmSubsystem;

public class ShooterAngleFromDistanceInterpolation extends Command {
  private final ShooterArmSubsystem shooterArmSubsystem = ShooterArmSubsystem.getInstance();
  private final Vision vision = Vision.getInstance(); 
  /** Creates a new ShooterAngleFromDistanceInterpolation. */
  public ShooterAngleFromDistanceInterpolation() {
    this.addRequirements(shooterArmSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterArmSubsystem.moveArmTo(shooterArmSubsystem.angleFromDistance(vision.DistanceFromTarget()));
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
