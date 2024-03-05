// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.basicCommands.TakeFeedCommands;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.takeFeed.TakeFeedSubsystem;
import frc.robot.subsystems.takeFeed.TakeFeedConstants.takeFeedConstants;

public class CollectUntilNote extends Command {
  
  TakeFeedSubsystem takeFeed = TakeFeedSubsystem.getInstance();
  boolean collected = false;
  boolean finished = false;
  Timer timer;

  public CollectUntilNote() {
    addRequirements(takeFeed);
    timer = new Timer();
    finished = false;
  }

  @Override
  public void initialize() {
    double robotVelocity = 
      Math.abs(SwerveSubsystem.getInstance().getRobotVelocity().vxMetersPerSecond) 
      + Math.abs(SwerveSubsystem.getInstance().getRobotVelocity().vyMetersPerSecond);

    if(RobotState.isAutonomous()){
      takeFeed.setSpeed(4);
    }
    else{
      takeFeed.setSpeed(robotVelocity>0.5? takeFeedConstants.COLLECT_SPEED:9);
    }

    collected = false;
    finished = false;
    timer.reset();
    timer.stop();
    takeFeed.setSpeed(takeFeedConstants.COLLECT_SPEED);
  }

  @Override
  public void execute() {
    if(!collected&&takeFeed.getOpticSensorValue()){
      takeFeed.setSpeed(-2);
      timer.start();
      collected = true;
    }

    if(collected&&timer.hasElapsed(0.05)&&takeFeed.getOpticSensorValue()){
      finished = true;
    }

  }
  
  @Override
  public void end(boolean interrupted) {
    takeFeed.setSpeed(0);
  }
  
  @Override
  public boolean isFinished() {
    // return takeFeed.getOpticSensorValue();
    return finished&&!takeFeed.getOpticSensorValue();
  }
}
