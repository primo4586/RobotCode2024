// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.a_robotCommandGroups;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.basicCommands.ShooterArmCommands.ShooterArmSpeakerAngle;
import frc.robot.basicCommands.ShooterCommands.ShooterSetSpeedForever;
import frc.robot.basicCommands.TakeFeedCommands.FeedToShooter;
import frc.robot.subsystems.shooter.ShooterConstants.shooterConstants;

public class ShootSpeaker extends ParallelDeadlineGroup {
  public ShootSpeaker() {
    super(Commands.waitSeconds(0.5).andThen(new FeedToShooter()));

    addCommands(
        new ShooterArmSpeakerAngle(),
        new ShooterSetSpeedForever(shooterConstants.SHOOT_SPEED));
  }
}
