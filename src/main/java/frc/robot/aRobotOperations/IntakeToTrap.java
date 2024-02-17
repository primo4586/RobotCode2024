// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.aRobotOperations;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.IntakeConstants;
import frc.robot.basicCommands.IntakeArmCommands.IntakeArmUP;
import frc.robot.basicCommands.IntakeCommands.IntakeSetSpeed;
import frc.robot.basicCommands.TrapCommands.TrapCollectUntilNote;

public class IntakeToTrap extends SequentialCommandGroup {
  /** Creates a new IntakeToTrap. */
  public IntakeToTrap() {
    addCommands(
        new IntakeArmUP(),
        new TrapCollectUntilNote()
            .alongWith(new IntakeSetSpeed(()->IntakeConstants.getNoteSpeed))
            .andThen(new IntakeSetSpeed(()-> 0))
    );
  }
}
