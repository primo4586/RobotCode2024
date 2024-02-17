// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.aRobotOperations;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.basicCommands.IntakeArmCommands.IntakeArmDown;
import frc.robot.basicCommands.IntakeArmCommands.IntakeArmUP;

public class IntakeToFeeder extends SequentialCommandGroup {

  /** Creates a new FeederIntakeCommandGroup. */

  public IntakeToFeeder() {

    addCommands(
        new IntakeArmDown(),
        new CollectToFeeder(),
        new IntakeArmUP());
  }
}
