// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.basicCommands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class InatkeUntilNote extends Command {
  private final IntakeSubsystem collectingSubsystem = IntakeSubsystem.getInstance();

  /** Creates a new setSpeedUntilFeedCommand. */
  public InatkeUntilNote() {
    addRequirements(collectingSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    collectingSubsystem.setSpeed(IntakeConstants.getNoteSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    collectingSubsystem.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (collectingSubsystem.getSwitch());
  }
}
