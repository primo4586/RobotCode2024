// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.basicCommands.TakeFeedCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.takeFeed.TakeFeedSubsystem;
import frc.robot.subsystems.takeFeed.TakeFeedConstants.takeFeedConstants;

public class CollectUntilNote extends Command {
  
  TakeFeedSubsystem takeFeed = TakeFeedSubsystem.getInstance();
  boolean collected = false;
  Timer timer;

  public CollectUntilNote() {
    addRequirements(takeFeed);
    timer = new Timer();
  }

  @Override
  public void initialize() {
    double robotVelocity = 
      Math.abs(SwerveSubsystem.getInstance().getRobotVelocity().vxMetersPerSecond) 
      + Math.abs(SwerveSubsystem.getInstance().getRobotVelocity().vyMetersPerSecond);


    takeFeed.setSpeed(robotVelocity>0.5? takeFeedConstants.COLLECT_SPEED:12);
    collected = false;
    timer.reset();
    timer.stop();
  }

  @Override
  public void execute() {
    if(!collected&&takeFeed.getOpticSensorValue()){
      takeFeed.setSpeed(-1);
      timer.start();
      collected = true;
    }
  }
  
  @Override
  public void end(boolean interrupted) {
    takeFeed.setSpeed(0);
  }
  
  @Override
  public boolean isFinished() {
    return collected&&timer.hasElapsed(0.05)&&takeFeed.getOpticSensorValue();
  }
}
