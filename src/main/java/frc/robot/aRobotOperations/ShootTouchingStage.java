// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.aRobotOperations;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.ShooterArmConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.basicCommands.ShooterArmCommands.MoveShooterArmTo;
import frc.robot.basicCommands.ShooterCommands.ShooterSetSpeed;
import frc.robot.basicCommands.feederCommands.FeedToShooter;

public class ShootTouchingStage extends ParallelCommandGroup {
  /** Creates a new ShootTouchingStage. */
  public ShootTouchingStage() {
    addCommands(
      new MoveShooterArmTo(ShooterArmConstants.ShootStageAngle),
      new ShooterSetSpeed(ShooterConstants.ShootStageSpeed),
      new FeedToShooter()
    );
  }
}
