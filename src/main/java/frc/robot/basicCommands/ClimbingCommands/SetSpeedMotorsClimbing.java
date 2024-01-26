// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.basicCommands.ClimbingCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ClimbingSubsystem;

public class SetSpeedMotorsClimbing extends InstantCommand {
  private final ClimbingSubsystem climbingSubsystem = ClimbingSubsystem.getInstance();
  double speed;

  public SetSpeedMotorsClimbing(double speed) {
    this.addRequirements(climbingSubsystem);
    this.speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.climbingSubsystem.setSpeedClimbing(() -> speed);

  }
}
