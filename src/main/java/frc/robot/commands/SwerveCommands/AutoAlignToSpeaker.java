// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SwerveCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants.Speaker;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.takeFeed.TakeFeedSubsystem;
import frc.utils.AllianceFlipUtil;
import frc.utils.vision.Vision;

public class AutoAlignToSpeaker extends Command {
  /** Creates a new TurnToDegree. */
  SwerveSubsystem swerve = SwerveSubsystem.getInstance();
  TakeFeedSubsystem feeder = TakeFeedSubsystem.getInstance();

  private Vision vision = Vision.getInstance();
  double angleFromTarget = vision.GetAngleFromTarget().getDegrees();
  
  Rotation2d heading = new Rotation2d();

  public AutoAlignToSpeaker() {
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    Pose2d robot = swerve.getPose();
    
    double deltaX = Math.abs(robot.getX() - AllianceFlipUtil.apply(Speaker.centerSpeakerOpening.getX()));
    double deltaY = Speaker.centerSpeakerOpening.getY() - robot.getY();
    double tempHeading = Units.radiansToDegrees(Math.atan2(deltaY, deltaX));
    if(DriverStation.getAlliance().get() == Alliance.Red)
      heading = new Rotation2d(Units.degreesToRadians(tempHeading + 180));
    else
      heading = new Rotation2d(Units.degreesToRadians(360-tempHeading));

    SmartDashboard.putNumber("heading", heading.getDegrees());
    swerve.setHeading(heading);
    swerve.drive(new Translation2d(),0,true,true,true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d robot = swerve.getPose();
    
    double deltaX = Math.abs(robot.getX() - AllianceFlipUtil.apply(Speaker.centerSpeakerOpening.getX()));
    double deltaY = Speaker.centerSpeakerOpening.getY() - robot.getY();
    double tempHeading = Units.radiansToDegrees(Math.atan2(deltaY, deltaX));
    if(DriverStation.getAlliance().get() == Alliance.Red)
      heading = new Rotation2d(Units.degreesToRadians(tempHeading + 180));
    else
      heading = new Rotation2d(Units.degreesToRadians(360-tempHeading));

    SmartDashboard.putNumber("heading", heading.getDegrees());
    swerve.setHeading(heading);
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
    return false;
  }
}
