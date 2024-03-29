// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.basicCommands.feederCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeArmSubsystem;

public class FeedUntilNote extends Command {
  private final FeederSubsystem feederSubsystem = FeederSubsystem.getInstance();

  /** Creates a new FeedUntilNote. */
  public FeedUntilNote() {
    addRequirements(feederSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    feederSubsystem.setSpeed(0.4);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    feederSubsystem.setSpeed(0);
    IntakeArmSubsystem.getInstance().setSpeed(()->0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return feederSubsystem.getSwitch();
  }
}
