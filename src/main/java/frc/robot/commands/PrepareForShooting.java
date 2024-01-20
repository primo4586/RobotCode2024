// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.ShooterArmCommands.ShooterAngleFromDistanceInterpolation;
import frc.robot.commands.ShooterCommands.ShooterSetSpeedInterpolation;
import frc.robot.subsystems.ShooterArmSubsystem;
import frc.robot.subsystems.ShooterSubsysem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PrepareForShooting extends ParallelCommandGroup {
  private final ShooterSubsysem shooterSubsysem = ShooterSubsysem.getInstance();
  private final ShooterArmSubsystem shooterArmSubsystem = ShooterArmSubsystem.getInstance();
  double degree;
  /** Creates a new PreperForShootting. */
  public PrepareForShooting(double degree) {
    this.addRequirements(shooterSubsysem, shooterArmSubsystem);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ShooterSetSpeedInterpolation(),
      new ShooterAngleFromDistanceInterpolation()
    );
  }
}
