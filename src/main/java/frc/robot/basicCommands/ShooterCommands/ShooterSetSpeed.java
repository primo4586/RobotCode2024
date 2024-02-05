// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.basicCommands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterSetSpeed extends InstantCommand {
  private final ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();
  double speed;

  /** Creates a new ShooterSetSpeed. */
  public ShooterSetSpeed(double speed) {
    this.addRequirements(shooterSubsystem);
    this.speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.shooterSubsystem.setSpeedShooterd(speed);
  }
}
