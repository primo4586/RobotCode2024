// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.feederCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.IntakeArmCommands.MoveIntakeArmToDegree;
import frc.robot.commands.IntakeCommands.CollectToFeeder;
import frc.robot.commands.IntakeCommands.IntakeSetSpeed;
import static frc.robot.Constants.IntakeArm.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FeederIntakeCommandGroup extends SequentialCommandGroup {

  /** Creates a new FeederIntakeCommandGroup. */
  
  public FeederIntakeCommandGroup() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new MoveIntakeArmToDegree(intakeSetPoint),
        new CollectToFeeder(),
        new IntakeSetSpeed(0),
        new MoveIntakeArmToDegree(trapSetPoint));
  }
}
