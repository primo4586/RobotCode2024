// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TakeFeedCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooterArm.ShooterArmSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.takeFeed.TakeFeedSubsystem;
import frc.robot.subsystems.takeFeed.TakeFeedConstants.takeFeedConstants;

public class FeedToShooter extends Command {

  private final TakeFeedSubsystem feeder = TakeFeedSubsystem.getInstance();
  private final SwerveSubsystem swerve = SwerveSubsystem.getInstance();
  private final ShooterArmSubsystem shooterArm = ShooterArmSubsystem.getInstance();
  private final ShooterSubsystem shooter = ShooterSubsystem.getInstance();
  private Timer timer;
  private boolean startedShooting;

  public FeedToShooter() {
    addRequirements(feeder);
    timer = new Timer();
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.stop();
  }

  @Override
  public void execute() {
    if (!startedShooting &&
        shooterArm.isArmReady() &&
        shooter.checkIfShooterAtSpeed() &&
        swerve.headingPid.atSetpoint()) {

      feeder.setSpeed(takeFeedConstants.SHOOT_SPEED);
      timer.restart();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    feeder.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(takeFeedConstants.FEED_SHOOTER_TIME);
  }
}
