// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.basicCommands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Leds.Leds;
import frc.robot.subsystems.Leds.LedsConstants;

public class LedsYay extends Command {

  private final Leds leds = Leds.getInstance();
  /** Creates a new LedsYay. */
  public LedsYay() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
        Color[] colors = new Color[LedsConstants.LedsLength];
        for (int i = 0; i < colors.length; i++) {
            final int hue = (i * 180 / colors.length) % 180;
            colors[i] = Color.fromHSV(hue, 255, 128);
            leds.turnLedsByRGBRange((int)colors[i].red * 255, (int)colors[i].green * 255, (int)colors[i].blue * 255, i, i);
          }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
