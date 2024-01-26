// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.aRobotOperations;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.basicCommands.ShooterArmCommands.ShooterArmSpeakerAngle;
import frc.robot.basicCommands.ShooterCommands.ShooterSpeakerSetSpeed;

public class PrepareForShoot extends ParallelCommandGroup {
  /** Creates a new PrepareForShoot. */
  public PrepareForShoot() {
    
    addCommands(
      new ShooterArmSpeakerAngle(),
      new ShooterSpeakerSetSpeed()
    );
  }
}
