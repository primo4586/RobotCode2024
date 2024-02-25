// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.aRobotOperations;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.ShooterArmConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.basicCommands.ShooterArmCommands.MoveShooterArmTo;
import frc.robot.basicCommands.ShooterArmCommands.ShooterArmSetSpeed;
import frc.robot.basicCommands.ShooterCommands.ShooterSetSpeed;
import frc.robot.basicCommands.feederCommands.FeedToShooter;
import frc.robot.basicCommands.feederCommands.FeedToShooterBase;

public class ShootTouchingBase extends ParallelCommandGroup {
  /** Creates a new ShootTouchingBase. */
  public ShootTouchingBase() {
    addCommands(
      new MoveShooterArmTo(ShooterArmConstants.ShootBaseAngle),
      new ShooterSetSpeed(ShooterConstants.ShootBaseSpeed),
      new FeedToShooterBase()
    );
  }
}
