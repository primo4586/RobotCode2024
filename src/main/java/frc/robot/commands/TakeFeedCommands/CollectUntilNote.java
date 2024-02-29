// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TakeFeedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.takeFeed.TakeFeedSubsystem;
import frc.robot.subsystems.takeFeed.TakeFeedConstants.takeFeedConstants;

public class CollectUntilNote extends Command {
  
  TakeFeedSubsystem takeFeed = TakeFeedSubsystem.getInstance();

  public CollectUntilNote() {
    addRequirements(takeFeed);
  }

  @Override
  public void initialize() {
    takeFeed.setSpeed(takeFeedConstants.COLLECT_SPEED);
  }
  
  @Override
  public void end(boolean interrupted) {
    takeFeed.setSpeed(0.0);
  }
  
  @Override
  public boolean isFinished() {
    return takeFeed.getOpticSensorValue();
  }
}
