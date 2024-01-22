// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimbingCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ClimbingSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetSpeedRightMotorClimbing extends InstantCommand {
  private final ClimbingSubsystem climbingSubsystem = ClimbingSubsystem.getInstance();
  double speed;
  public SetSpeedRightMotorClimbing(double speed) {
    this.addRequirements(climbingSubsystem);
    this.speed = speed;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.climbingSubsystem.setSpeedClimbing(speed);

  }
}
