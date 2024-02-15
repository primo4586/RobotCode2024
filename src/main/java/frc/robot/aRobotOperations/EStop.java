// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.aRobotOperations;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.basicCommands.ClimbingCommands.ClimbingSetSpeed;
import frc.robot.basicCommands.IntakeArmCommands.IntakeArmSetSpeed;
import frc.robot.basicCommands.IntakeCommands.IntakeSetSpeed;
import frc.robot.basicCommands.ShooterCommands.ShooterSetSpeed;
import frc.robot.basicCommands.TrapArmCommands.TrapArmSetSpeed;
import frc.robot.basicCommands.feederCommands.FeederSetSpeed;

public class EStop extends ParallelCommandGroup {

  public EStop() {
    addCommands(
      new ClimbingSetSpeed(0),
      new FeederSetSpeed(()->0),
      new IntakeArmSetSpeed(()->0),
      new IntakeSetSpeed(()->0),
      new ShooterSetSpeed(0),
      new TrapArmSetSpeed(0),
      new TrapArmSetSpeed(0)
    );
  }
}
