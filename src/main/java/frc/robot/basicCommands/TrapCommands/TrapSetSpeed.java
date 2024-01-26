// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.basicCommands.TrapCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.TrapSubsystem;

public class TrapSetSpeed extends InstantCommand {
  private final TrapSubsystem subsystemTrap = TrapSubsystem.getInstance();
  double speed;

  public TrapSetSpeed(double speed) {
    this.addRequirements(subsystemTrap);
    this.speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.subsystemTrap.setSpeed(speed);
  }
}
