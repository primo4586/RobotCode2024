// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.basicCommands.ShooterArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooterArm.ShooterArmSubsystem;
import frc.robot.subsystems.shooterArm.ShooterArmConstants.shooterArmConstants;

public class ZeroShooterArm extends Command {
  /** Creates a new ZeroShooterArm. */
  private final ShooterArmSubsystem shooterArm = ShooterArmSubsystem.getInstance();
private boolean zerodOut = false;
private boolean canReset = false;

  public ZeroShooterArm() {
    addRequirements(shooterArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    canReset = !shooterArm.getSwitch();
    zerodOut = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!canReset){
      shooterArm.setSpeed(-shooterArmConstants.RESET_SPEED);
    }
    
    if(!shooterArm.getSwitch()){
      shooterArm.setSpeed(shooterArmConstants.RESET_SPEED);
      canReset = true;
    }
    else if(canReset){
      zerodOut = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterArm.setPosition(shooterArmConstants.RESET_POSE);
    shooterArm.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return zerodOut;
  }
}
