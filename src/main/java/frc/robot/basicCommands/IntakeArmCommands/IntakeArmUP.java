// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.basicCommands.IntakeArmCommands;

import static frc.robot.Constants.IntakeArmConstants.intakeArmUpSpeed;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeArmSubsystem;

public class IntakeArmUP extends Command {
  /** Creates a new IntakeArmUP. */
private final IntakeArmSubsystem intakeArm = IntakeArmSubsystem.getInstance();

  public IntakeArmUP() {
    addRequirements(intakeArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeArm.setSpeed(() -> intakeArmUpSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeArm.setSpeed(() -> 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intakeArm.getUpSwitch();
  }
}
