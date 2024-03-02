// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.a_robotCommandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.basicCommands.ShooterArmCommands.ShooterArmMoveToAngle;
import frc.robot.basicCommands.ShooterCommands.ShooterSetSpeed;
import frc.robot.subsystems.shooter.ShooterConstants.shooterConstants;
import frc.robot.subsystems.shooterArm.ShooterArmConstants.shooterArmConstants;

public class ReadyForAmp extends ParallelCommandGroup {
  public ReadyForAmp() {
    addCommands(
      new ShooterArmMoveToAngle(shooterArmConstants.AMP_ANGLE),
      new ShooterSetSpeed(shooterConstants.AMP_SPEED)
    );
  }
}
