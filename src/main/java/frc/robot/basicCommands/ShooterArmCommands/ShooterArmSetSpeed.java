// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.basicCommands.ShooterArmCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ShooterArmSubsystem;
import java.util.function.DoubleSupplier;

public class ShooterArmSetSpeed extends InstantCommand {
  ShooterArmSubsystem shooterArm = ShooterArmSubsystem.getInstance();
  DoubleSupplier speed;
  public ShooterArmSetSpeed(DoubleSupplier speed) {
    addRequirements(shooterArm);
    this.speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterArm.setSpeedArm(speed);
  }
}
