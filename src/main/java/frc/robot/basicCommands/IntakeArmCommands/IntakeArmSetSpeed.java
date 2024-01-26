// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.basicCommands.IntakeArmCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IntakeArmSubsystem;

public class IntakeArmSetSpeed extends InstantCommand {
  private final IntakeArmSubsystem intakeArm = IntakeArmSubsystem.getInstance();
  private double speed;

  public IntakeArmSetSpeed(double speed) {
    addRequirements(intakeArm);
    this.speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeArm.setSpeed(() -> speed);
  }
}
