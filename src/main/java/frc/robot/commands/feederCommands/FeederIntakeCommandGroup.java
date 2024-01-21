// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.feederCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.CollectingCommands.setSpeedCommand;
import frc.robot.commands.CollectingCommands.setSpeedUntilFeedCommand;
import frc.robot.commands.IntakeArmCommands.MoveIntakeArmToDegree;
import frc.robot.subsystems.CollectingSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeArmSubsystem;
import static frc.robot.Constants.IntakeArm.*;
import static frc.robot.Constants. FeederConstants.*;
import static frc.robot.Constants. CollectingConstants.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FeederIntakeCommandGroup extends SequentialCommandGroup {

  /** Creates a new FeederIntakeCommandGroup. */
  private static IntakeArmSubsystem intakeArmSubsystem = IntakeArmSubsystem.getInstance();
  private static FeederSubsystem feederSubsystem = FeederSubsystem.getInstance();
  private static CollectingSubsystem collectingSubsystem = CollectingSubsystem.getInstance();
  
  
  public FeederIntakeCommandGroup() {
    addRequirements(intakeArmSubsystem);
    addRequirements(feederSubsystem);
    addRequirements(collectingSubsystem);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new MoveIntakeArmToDegree(intakeDegree), new setSpeedUntilFeedCommand(collectingSpeed),new MoveIntakeArmToDegree(feederDegree) ,new FeedShooter(), new setSpeedCommand(collectingGivingSpeed),new setSpeedCommand(0),new FeederSetSpeed(0)); // wating for feeder switch  
  }
}
