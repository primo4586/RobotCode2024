// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.basicCommands.SwerveCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class AlignToAngle extends Command {
/** Creates a new TurnToDegree. */
  SwerveSubsystem swerve = SwerveSubsystem.getInstance();

  
  Rotation2d heading = new Rotation2d();

  public AlignToAngle(Rotation2d angle) {
    heading = angle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swerve.setHeading(heading);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerve.drive(new Translation2d(),0,true,true,true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.disableHeading();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return swerve.headingPid.atSetpoint();
  }
}