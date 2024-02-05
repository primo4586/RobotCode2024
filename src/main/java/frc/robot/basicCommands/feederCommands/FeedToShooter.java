// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.basicCommands.feederCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FeederConstants;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterArmSubsystem;
import frc.robot.subsystems.ShooterSubsysem;
import static frc.robot.Constants.FeederConstants.*;

public class FeedToShooter extends Command {
  private final ShooterSubsysem shooterSubsystem = ShooterSubsysem.getInstance();
  private final ShooterArmSubsystem shooterArmSubsystem = ShooterArmSubsystem.getInstance();
  private final FeederSubsystem feederSubsystem = FeederSubsystem.getInstance();
  Timer timer = new Timer();
  boolean startedShooting = false;

  /** Creates a new StartFeeder. */
  public FeedToShooter() {
    this.addRequirements(feederSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
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
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(FeederConstants.TimeToFeed);
  }
}
