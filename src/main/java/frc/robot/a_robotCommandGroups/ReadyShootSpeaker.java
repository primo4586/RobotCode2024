// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.a_robotCommandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.basicCommands.ShooterArmCommands.ShooterArmSpeakerAngle;
import frc.robot.basicCommands.ShooterCommands.ShooterSetSpeedForever;
import frc.robot.basicCommands.TakeFeedCommands.TakeFeedSetSpeedForever;
import frc.robot.subsystems.shooter.ShooterConstants.shooterConstants;

public class ReadyShootSpeaker extends ParallelCommandGroup {
  public ReadyShootSpeaker() {
    addCommands(
        new ShooterArmSpeakerAngle(),
        new ShooterSetSpeedForever(shooterConstants.SHOOT_SPEED));
  }
}
