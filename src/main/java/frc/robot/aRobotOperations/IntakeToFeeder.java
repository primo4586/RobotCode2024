// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.aRobotOperations;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.basicCommands.IntakeArmCommands.MoveIntakeArmToDegree;
import static frc.robot.Constants.IntakeArmConstants.*;

public class IntakeToFeeder extends SequentialCommandGroup {

  /** Creates a new FeederIntakeCommandGroup. */

  public IntakeToFeeder() {

    addCommands(
        new MoveIntakeArmToDegree(intakeSetPoint),
        new CollectToFeeder(),
        new MoveIntakeArmToDegree(trapSetPoint));
  }
}
