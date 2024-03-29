// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.basicCommands.TrapCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.trapConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TrapSubsystem;

public class TrapCollectUntilNote extends Command {
  private final TrapSubsystem trap = TrapSubsystem.getInstance();
  private final IntakeSubsystem intake = IntakeSubsystem.getInstance();

  /** Creates a new TrapCollectUntilNote. */
  public TrapCollectUntilNote() {
    addRequirements(trap, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    trap.setSpeed(trapConstants.TrapCollectSpeed);
    intake.setSpeed(IntakeConstants.feedToTrapSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    trap.setSpeed(0);
    intake.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return trap.isNoteDetected();
  }
}
