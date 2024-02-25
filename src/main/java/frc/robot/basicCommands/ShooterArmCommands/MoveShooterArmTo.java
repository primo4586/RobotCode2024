// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.basicCommands.ShooterArmCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ShooterArmSubsystem;

public class MoveShooterArmTo extends InstantCommand {
  /** Creates a new MoveShooterArmToCommand. */
  private final ShooterArmSubsystem shooterArmSubsystem = ShooterArmSubsystem.getInstance();
  double degree;

  public MoveShooterArmTo(double degree) {
    addRequirements(shooterArmSubsystem);
    this.degree = degree;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterArmSubsystem.moveArmTo(degree);
  }
}
