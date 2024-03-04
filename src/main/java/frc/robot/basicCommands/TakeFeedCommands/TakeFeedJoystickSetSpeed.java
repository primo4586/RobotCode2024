// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.basicCommands.TakeFeedCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.takeFeed.TakeFeedSubsystem;

public class TakeFeedJoystickSetSpeed extends Command {
  
  TakeFeedSubsystem takeFeed = TakeFeedSubsystem.getInstance();

  DoubleSupplier input;

  public TakeFeedJoystickSetSpeed(DoubleSupplier input) {
    addRequirements(takeFeed);
    this.input = input;
  }
  @Override
  public void execute() {
    takeFeed.setSpeed(input.getAsDouble() * 12);
  }

  @Override
  public void end(boolean interrupted) {
    takeFeed.setSpeed(0.0);
  }

}
