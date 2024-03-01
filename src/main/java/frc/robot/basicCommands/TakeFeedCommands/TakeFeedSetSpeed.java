// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.basicCommands.TakeFeedCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.takeFeed.TakeFeedSubsystem;

public class TakeFeedSetSpeed extends InstantCommand {
  
  TakeFeedSubsystem takeFeed = TakeFeedSubsystem.getInstance();

  double voltage;

  public TakeFeedSetSpeed(double voltage) {
    addRequirements(takeFeed);
    this.voltage = voltage;
  }

  @Override
  public void initialize() {
    takeFeed.setSpeed(voltage);
  }
}
