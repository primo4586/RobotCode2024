// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeArmSubsystem;
import static frc.robot.Constants.IntakeArm.*;

public class IntakeZero extends Command {
  /** Creates a new SetSpeed. */
  private final IntakeArmSubsystem intakeArm = IntakeArmSubsystem.getInstance();
  public IntakeZero() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeArm.setSpeed(intakeArmSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeArm.setSpeed(0);
    intakeArm.setEncoder(zeroEncoder);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intakeArm.getSwitch();
  }
}
