// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.basicCommands.LedsCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Leds.Leds;
import frc.robot.subsystems.takeFeed.TakeFeedSubsystem;

public class BasicLedsCommand extends Command {

  private final TakeFeedSubsystem takeFeed = TakeFeedSubsystem.getInstance();
  private final Leds leds = Leds.getInstance();
  /** Creates a new BasicLedsCommand. */
  public BasicLedsCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    this.addRequirements(leds);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (takeFeed.getOpticSensorValue()){
      leds.LedsRainbow(15);
    }
    else {
      leds.waveEffect(15);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
