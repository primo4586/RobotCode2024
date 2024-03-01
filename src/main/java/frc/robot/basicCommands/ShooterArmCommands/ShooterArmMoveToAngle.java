// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.basicCommands.ShooterArmCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.shooterArm.ShooterArmSubsystem;

public class ShooterArmMoveToAngle extends InstantCommand {

  private final ShooterArmSubsystem shooterArm = ShooterArmSubsystem.getInstance();
  double angle;
  
  public ShooterArmMoveToAngle(double angle) {
    addRequirements(shooterArm);
    this.angle = angle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterArm.moveArmTo(angle);
  }
}
