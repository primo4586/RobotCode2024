// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.aRobotOperations;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.basicCommands.IntakeArmCommands.IntakeArmDown;
import frc.robot.basicCommands.IntakeCommands.InatkeUntilNote;

public class CollectToIntake extends ParallelCommandGroup {
  /** Creates a new CollectToIntake. */
  public CollectToIntake() {
    addCommands(
        new IntakeArmDown(),
        new InatkeUntilNote());
  }
}
