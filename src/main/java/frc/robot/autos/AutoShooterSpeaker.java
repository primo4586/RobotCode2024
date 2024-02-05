// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.FeederConstants;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import static frc.robot.Constants.FeederConstants.*;

public class AutoShooterSpeaker extends Command {
  private final ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();
  private final SwerveSubsystem swerve = SwerveSubsystem.getInstance();
  private final ShooterArmSubsystem shooterArmSubsystem = ShooterArmSubsystem.getInstance();
  private final FeederSubsystem feederSubsystem = FeederSubsystem.getInstance();
  Timer timer = new Timer();
  boolean startedShooting = false;

  public AutoShooterSpeaker() {
    this.addRequirements(shooterSubsystem, shooterArmSubsystem, feederSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  private Pose2d pose;
  private Rotation2d directionOfTravel;
  private double translationX;
  private double translationY;
  private Transform2d heading;
  double translationMagnitude = AutoConstants.AutoShootSpeed * FeederConstants.TimeToFeed;

  @Override
  public void execute() {
    pose = swerve.getPose();
    directionOfTravel = swerve.headingSupplier.get();
    translationX = translationMagnitude * Math.cos(directionOfTravel.getRadians());
    translationY = translationMagnitude * Math.sin(directionOfTravel.getRadians());
    heading = new Transform2d(translationX, translationY, new Rotation2d());
    pose = pose.transformBy(heading);
    shooterSubsystem.setSpeedShooter(shooterSubsystem.speakerInterpolate(pose));
    shooterArmSubsystem.moveArmTo(shooterArmSubsystem.angleFromDistance(swerve.getPose()));

    if (!startedShooting && shooterSubsystem.checkIfShooterAtSpeed() && shooterArmSubsystem.isArmReady()) {
      feederSubsystem.setSpeed(FeederShootSpeed);
      startedShooting = true;
      timer.restart();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    feederSubsystem.setSpeed(0);
    shooterSubsystem.coast();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
