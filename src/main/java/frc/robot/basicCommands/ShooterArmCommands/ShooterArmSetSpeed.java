// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.basicCommands.ShooterArmCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.shooterArm.ShooterArmSubsystem;

public class ShooterArmSetSpeed extends InstantCommand {

  private final ShooterArmSubsystem shooterArm = ShooterArmSubsystem.getInstance();
  double speed;

  public ShooterArmSetSpeed(double speed) {
    addRequirements(shooterArm);
    this.speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterArm.setSpeed(speed);
  }
}
