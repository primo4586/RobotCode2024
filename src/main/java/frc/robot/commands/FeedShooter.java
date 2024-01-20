// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.FeederSubsystem;

import frc.robot.Constants.FeederConstants;;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FeedShooter extends InstantCommand {

  // feeder subsystem
  private FeederSubsystem feeder;

  // constructor
  public FeedShooter(FeederSubsystem feeder) {
    addRequirements(feeder);
    this.feeder = feeder;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // set speed to motor
    feeder.setSpeed(FeederConstants.FeederMotorSpeed);

  }
}
