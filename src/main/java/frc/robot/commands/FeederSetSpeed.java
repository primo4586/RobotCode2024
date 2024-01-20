// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.FeederSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FeederSetSpeed extends InstantCommand {
  // variables
  private FeederSubsystem feeder;// feeder subsystem
  double speed;

  // constructor
  public FeederSetSpeed(FeederSubsystem feeder, double speed) {
    addRequirements(feeder);
    this.feeder = feeder;
    this.speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // set speed to motor
    feeder.setSpeed(speed);
  }
}
