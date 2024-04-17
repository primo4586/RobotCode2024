// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.a_robotCommandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.basicCommands.ShooterArmCommands.ShooterArmSpeakerAngle;
import frc.robot.basicCommands.ShooterArmCommands.ShooterArmTag;
import frc.robot.basicCommands.SwerveCommands.AutoAlignToTagWhileTrue;
import frc.robot.basicCommands.SwerveCommands.TurnToTag;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlignTag extends ParallelCommandGroup {
  /** Creates a new AlignTag. */
  public AlignTag() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new TurnToTag(),
      new ShooterArmTag()
    );
  }
}
