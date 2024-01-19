// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterArmSubsystem;

import static frc.robot.Constants.ShooterArmConstants.*;


public class MoveShooterArmUntil extends Command {
  /** Creates a new ResetShooterArmPos. */
  private final ShooterArmSubsystem shooterArmSubsystem = ShooterArmSubsystem.getInstance();
  
  public MoveShooterArmUntil() {
    addRequirements(shooterArmSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterArmSubsystem.moveArmBySpeed(resetSpeed);
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterArmSubsystem.setPosition(startPose);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shooterArmSubsystem.getSwitch();
  }
}
