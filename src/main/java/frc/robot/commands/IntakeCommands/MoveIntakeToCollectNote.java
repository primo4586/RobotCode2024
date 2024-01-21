// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.IntakeArmCommands.MoveIntakeArmToDegree;
import frc.robot.commands.feederCommands.FeedUntilNote;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MoveIntakeToCollectNote extends ParallelCommandGroup {
  /** Creates a new MoveIntakeToCollectNote. */
  public MoveIntakeToCollectNote() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new IntakeSetSpeed(IntakeConstants.getNoteSpeed),
      new FeedUntilNote(),
      new MoveIntakeArmToDegree(IntakeConstants.GroundIntakePose));
  }
}
