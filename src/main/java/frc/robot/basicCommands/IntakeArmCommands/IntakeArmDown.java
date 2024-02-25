// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.basicCommands.IntakeArmCommands;

import static frc.robot.Constants.IntakeArmConstants.intakeSetPoint;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeArmSubsystem;

public class IntakeArmDown extends Command {
  /** Creates a new IntakeArmDown. */
private final IntakeArmSubsystem intakeArm = IntakeArmSubsystem.getInstance();

  public IntakeArmDown() {
    addRequirements(intakeArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeArm.moveArmTo(intakeSetPoint);
  }
  @Override
  public void end(boolean interrupted) {
    intakeArm.setSpeed(()->0);
    intakeArm.coast();
  }

  @Override
  public boolean isFinished() {
      return Math.abs(intakeArm.getPose() + 100)<10;
  }

}
