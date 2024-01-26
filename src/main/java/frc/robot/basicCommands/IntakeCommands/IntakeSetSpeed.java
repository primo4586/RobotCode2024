// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.basicCommands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeSetSpeed extends InstantCommand {
  private final IntakeSubsystem collectingSubsystem = IntakeSubsystem.getInstance();
  double speed;

  /** Creates a new setSpeedCommand. */
  public IntakeSetSpeed(double speed) {
    addRequirements(collectingSubsystem);
    this.speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    collectingSubsystem.setSpeed(speed);
  }
}
