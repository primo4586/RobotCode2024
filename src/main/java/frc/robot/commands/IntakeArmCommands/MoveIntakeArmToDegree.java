// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeArmSubsystem;

public class MoveIntakeArmToDegree extends Command {
  /** Creates a new MoveIntakeArmTo. */
  private final IntakeArmSubsystem intakeArm = IntakeArmSubsystem.getInstance();
  private double degree;

  public MoveIntakeArmToDegree(double degree) {
    this.degree = degree;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeArm.moveArmTo(degree);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intakeArm.checkIntakeArmPosion();
  }
}
