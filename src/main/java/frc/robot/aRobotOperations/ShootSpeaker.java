// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.aRobotOperations;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.basicCommands.feederCommands.FeedToShooter;
public class ShootSpeaker extends ParallelDeadlineGroup {
  /** Creates a new shootSpeker. */
  public ShootSpeaker() {
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().
    super(new FeedToShooter());
    addCommands(new PrepareForShoot());
    // addCommands(new FooCommand(), new BarCommand());
  }
}
