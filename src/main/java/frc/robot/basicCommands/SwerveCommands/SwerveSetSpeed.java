// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.basicCommands.SwerveCommands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class SwerveSetSpeed extends Command {
  private SwerveSubsystem swerve = SwerveSubsystem.getInstance();
  private double translation;
  private double strafe;
  private double rotation;
  private Boolean fieldRelative;
  private boolean isOpenLoop;

  public SwerveSetSpeed(double translation, double strafe, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    addRequirements(swerve);

    this.translation = translation;
    this.strafe = strafe;
    this.rotation = rotation;
    this.fieldRelative = fieldRelative;
    this.isOpenLoop = isOpenLoop;
  }

  @Override
  public void initialize() {
    swerve.drive(new Translation2d(translation, strafe), rotation, fieldRelative, isOpenLoop);
  }

  @Override
  public void end(boolean interrupted) {
    swerve.drive(new Translation2d(translation, strafe), rotation, fieldRelative, isOpenLoop);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
