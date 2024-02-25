// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.basicCommands.SwerveCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.Utils.vision.Vision;
import frc.robot.basicCommands.SwerveCommands.FieldConstants.Speaker;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AlignToSpeaker extends Command {
  /** Creates a new TurnToDegree. */
  SwerveSubsystem swerve = SwerveSubsystem.getInstance();
  FeederSubsystem feeder = FeederSubsystem.getInstance();

  private Vision vision = Vision.getInstance();
  double angleFromTarget = vision.GetAngleFromTarget().getDegrees();
  
  Rotation2d heading = new Rotation2d();

  public AlignToSpeaker() {
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d robot = swerve.getPose();
    
    double deltaX = Math.abs(robot.getX() - Speaker.centerSpeakerOpening.getX());
    double deltaY = Speaker.centerSpeakerOpening.getY() - robot.getY();
    double tempHeading = Units.radiansToDegrees(Math.atan2(deltaY, deltaX));
    if(DriverStation.getAlliance().get() == Alliance.Red)
      heading = new Rotation2d(Units.degreesToRadians(tempHeading + 180));
    else
      heading = new Rotation2d(Units.degreesToRadians(tempHeading));

    SmartDashboard.putNumber("heading", heading.getDegrees());
    swerve.setHeading(heading);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.disableHeading();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
