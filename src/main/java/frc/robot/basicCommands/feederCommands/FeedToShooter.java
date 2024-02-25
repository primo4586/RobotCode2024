// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.basicCommands.feederCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import static frc.robot.Constants.FeederConstants.*;

public class FeedToShooter extends Command {
  private final ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();
  private final ShooterArmSubsystem shooterArmSubsystem = ShooterArmSubsystem.getInstance();
  private final FeederSubsystem feederSubsystem = FeederSubsystem.getInstance();
  private final SwerveSubsystem swerve = SwerveSubsystem.getInstance();
  boolean startedShooting;
  Timer timer;

  /** Creates a new StartFeeder. */
  public FeedToShooter() {
    this.addRequirements(feederSubsystem);
    timer = new Timer();
    startedShooting = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    feederSubsystem.setSpeed(0);
    startedShooting = false;
    timer.reset();
    timer.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!startedShooting &&
        swerve.headingPid.getPositionError() < 1 &&
        shooterSubsystem.checkIfShooterAtSpeed() &&
        shooterArmSubsystem.isArmReady()) {

      feederSubsystem.setSpeed(FeederShootSpeed);
      startedShooting = true;
      timer.start();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    feederSubsystem.setSpeed(0);
    startedShooting = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return startedShooting && timer.hasElapsed(0.2);
  }
}
