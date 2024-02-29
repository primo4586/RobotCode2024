// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimbCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.ClimbingSubsystem;

import java.util.function.DoubleSupplier;

public class ClimbSetSpeed extends Command {

  ClimbingSubsystem climb = ClimbingSubsystem.getInstance();
  DoubleSupplier speed;

  public ClimbSetSpeed(DoubleSupplier speed) {
    addRequirements(climb);
    this.speed = speed;
  }

  @Override
  public void execute() {
    climb.setSpeed(speed.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    climb.setSpeed(0);
  }
}
