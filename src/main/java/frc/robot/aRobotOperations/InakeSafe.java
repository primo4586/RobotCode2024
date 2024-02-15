// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.aRobotOperations;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.IntakeArmConstants;
import frc.robot.basicCommands.IntakeArmCommands.MoveIntakeArmToDegree;
import frc.robot.basicCommands.IntakeCommands.IntakeSetSpeed;

public class InakeSafe extends ParallelCommandGroup {
  /** Creates a new InakeSafe. */
  public InakeSafe() {
    addCommands(
      new IntakeSetSpeed(()->0),
      new MoveIntakeArmToDegree(IntakeArmConstants.SafeSetPoint)
    );
  }
}
