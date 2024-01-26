// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.aRobotOperations;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.IntakeArmConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.basicCommands.IntakeArmCommands.MoveIntakeArmToDegree;
import frc.robot.basicCommands.IntakeCommands.IntakeSetSpeed;
import frc.robot.basicCommands.TrapArmCommands.goInSwitchCommand;
import frc.robot.basicCommands.TrapCommands.TrapCollectUntilNote;

public class IntakeToTrap extends SequentialCommandGroup {
  /** Creates a new IntakeToTrap. */
  public IntakeToTrap() {
    addCommands(
        new MoveIntakeArmToDegree(IntakeArmConstants.trapSetPoint)
            .alongWith(new goInSwitchCommand()),
        new TrapCollectUntilNote()
            .alongWith(new IntakeSetSpeed(IntakeConstants.getNoteSpeed))
    );
  }
}
