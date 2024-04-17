// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.basicCommands.SwerveCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.swerve.SwerveConstants.swerveConstants;
import frc.util.vision.Vision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TurnToTag extends Command {
  SwerveSubsystem swerve = SwerveSubsystem.getInstance();
  Vision vision = Vision.getInstance();
  public PIDController notepid = swerveConstants.notePid;
  public TurnToTag() {
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    double angelFromTag = vision.GetAngelFromTag();
    swerve.drive(new Translation2d(),notepid.calculate(angelFromTag,0),true,false);
  }
}
