// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.basicCommands.ShooterArmCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooterArm.ShooterArmSubsystem;

public class ShooterArmSpeakerAngle extends Command {

  private final ShooterArmSubsystem shooterArm = ShooterArmSubsystem.getInstance();
  double angle;

  public ShooterArmSpeakerAngle() {
    addRequirements(shooterArm);
  }

  @Override
  public void initialize() {
    SmartDashboard.putNumber("angle", shooterArm.speakerInterpolate());
    shooterArm.moveArmTo(shooterArm.speakerInterpolate());
  }

  @Override
  public void execute() {
    SmartDashboard.putNumber("angle", shooterArm.speakerInterpolate());
    shooterArm.moveArmTo(shooterArm.speakerInterpolate());
  }

  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
