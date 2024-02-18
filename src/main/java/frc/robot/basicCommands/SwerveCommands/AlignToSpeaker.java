// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.basicCommands.SwerveCommands;

import static frc.robot.Constants.Swerve.*;

import org.photonvision.estimation.RotTrlTransform3d;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.Utils.vision.Vision;
import frc.robot.Robot;
import frc.robot.basicCommands.SwerveCommands.FieldConstants.Speaker;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AlignToSpeaker extends Command {
  /** Creates a new TurnToDegree. */
  SwerveSubsystem swerve = SwerveSubsystem.getInstance();
  FeederSubsystem feeder = FeederSubsystem.getInstance();

  private PIDController pid = aligningPID;
  private Vision vision = Vision.getInstance();
  double angleFromTarget = vision.GetAngleFromTarget().getDegrees();
  
  Rotation2d heading = new Rotation2d();

  public AlignToSpeaker() {
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swerve.setHeadingCommand(()->heading);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d robot = AllianceFlipUtil.apply(swerve.getPose());
    
    double deltaX = Math.abs(robot.getX() - Speaker.centerSpeakerOpening.getX());
    double deltaY = Math.abs(robot.getY() - Speaker.centerSpeakerOpening.getY());
    double tempHeading = Units.radiansToDegrees(Math.atan2(deltaY, deltaX));
    if (deltaY > 0) {
      tempHeading = 360 - tempHeading;
    }
    
    heading = new Rotation2d(Units.degreesToRadians(tempHeading));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.disableHeading();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Robot.lastShootTimeSeconds - (RobotController.getFPGATime()/1000000) < 0.1;
  }
}
