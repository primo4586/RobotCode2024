// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.aRobotOperations;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.basicCommands.feederCommands.FeedToShooter;

public class ShootSpeaker extends ParallelCommandGroup {
  /** Creates a new ShootSpeaker. */
  public ShootSpeaker() {
    addCommands(
      new PrepareForShoot(),
      new FeedToShooter()
    );
  }
}
