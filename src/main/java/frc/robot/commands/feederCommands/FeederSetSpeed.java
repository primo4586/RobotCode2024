// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.feederCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.FeederSubsystem;

public class FeederSetSpeed extends InstantCommand {
  // variables
  private FeederSubsystem feeder = FeederSubsystem.getInstance();// feeder subsystem
  double speed;

  // constructor
  public FeederSetSpeed(double speed) {
    addRequirements(feeder);
    this.speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // set speed to motor
    feeder.setSpeed(speed);
  }
}
