// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CollectingCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CollectingConstants;
import frc.robot.subsystems.CollectingSubsystem;

public class setSpeedUntilFeedCommand extends Command {
  private final CollectingSubsystem collectingSubsystem = CollectingSubsystem.getInstance();
  /** Creates a new setSpeedUntilFeedCommand. */
  public setSpeedUntilFeedCommand() {
    addRequirements(collectingSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    collectingSubsystem.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (collectingSubsystem.getSwitch()){
      return true;
    }
    else{
      return false;
    }

  }
}
