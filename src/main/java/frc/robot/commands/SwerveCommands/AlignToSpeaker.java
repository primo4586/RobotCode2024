// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SwerveCommands;

import static frc.robot.Constants.IntakeArm.minimumError;
import static frc.robot.Constants.Swerve.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.Utils.vision.Vision;
import frc.robot.commands.SwerveCommands.*;
import frc.robot.subsystems.SwerveSubsystem;

public class AlignToSpeaker extends Command {
  /** Creates a new TurnToDegree. */
  SwerveSubsystem swerve = SwerveSubsystem.getInstance();
  private PIDController pid = swervePID;
  private Vision vision = Vision.getInstance();
  double angleFromTarget = vision.GetAngleFromTarget().getDegrees();

  public AlignToSpeaker() {

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    angleFromTarget = vision.GetAngleFromTarget().getDegrees();
    optimize();
    double rotation = pid.calculate(angleFromTarget, 180);
    swerve.drive(new Translation2d(), rotation, true, false, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(vision.GetAngleFromTarget().getDegrees() - 180) < minimumErrorAligning ;
  }

  public void optimize() {
    double delta = 180 - angleFromTarget;
    if (Math.abs(delta) > 90){
        angleFromTarget = delta > 90 ? (angleFromTarget -= 180) : (angleFromTarget += 180);
    }
  }
}
