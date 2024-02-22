// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.aRobotOperations;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.basicCommands.IntakeArmCommands.IntakeArmDown;
import frc.robot.basicCommands.IntakeArmCommands.IntakeArmUP;
import frc.robot.basicCommands.IntakeCommands.ReverseIntakeUntilNote;
import frc.robot.basicCommands.feederCommands.FeederSetSpeed;

public class FeederToIntake extends SequentialCommandGroup {
  /** Creates a new FeederToIntake. */
  public FeederToIntake() {
    addCommands(
        new IntakeArmDown(),
        new FeederSetSpeed(() -> -0.5),
        new ReverseIntakeUntilNote()
            .andThen(new ParallelCommandGroup(new IntakeArmUP().asProxy(), new FeederSetSpeed(() -> 0).asProxy())));
  }
}
