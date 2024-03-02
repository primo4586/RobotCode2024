// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.basicCommands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ShooterSetSpeedForever extends Command {
  
  ShooterSubsystem shooter = ShooterSubsystem.getInstance();
  double speed;

  public ShooterSetSpeedForever(double speed) {
    addRequirements(shooter);
    this.speed = speed;
  }

  @Override
  public void initialize() {
    shooter.setShooterSpeed(speed);
  }

  @Override
  public void end(boolean interrupted) {
    shooter.coast();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
