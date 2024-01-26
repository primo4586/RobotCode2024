// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.basicCommands.TrapArmCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.TrapArmSubsystem;

public class setSpeedCommand extends InstantCommand {
  private final TrapArmSubsystem trapArmSubsystem = TrapArmSubsystem.getInstance();
  double speed;

  /** Creates a new setSpeedCommand. */
  public setSpeedCommand(double speed) {
    addRequirements(trapArmSubsystem);
    this.speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    trapArmSubsystem.setSpeed(() -> speed);
  }
}
