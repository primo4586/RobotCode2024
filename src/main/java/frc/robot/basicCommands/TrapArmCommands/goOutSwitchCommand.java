// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.basicCommands.TrapArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TrapArmConstants;
import frc.robot.subsystems.TrapArmSubsystem;

public class goOutSwitchCommand extends Command {
  private final TrapArmSubsystem trapArmSubsystem = TrapArmSubsystem.getInstance();

  /** Creates a new goOutSwitch. */
  public goOutSwitchCommand() {
    addRequirements(trapArmSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    trapArmSubsystem.setSpeed(() -> TrapArmConstants.GoOutSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    trapArmSubsystem.setSpeed(() -> 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return trapArmSubsystem.getOuterSwitch();
  }
}
