// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.Utils.shuffleboardAlike;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import frc.Utils.PathPlanner.PathPlannerHelper;
import frc.Utils.feedForward.FeedForwardCharacterization;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.ShooterArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

/** Add your docs here. */
public class AutoContainer {
    private CommandSelector autoSelector;
    private Map<String, Command> autoPaths;
    private PathPlannerHelper pathPlanner;
    SwerveSubsystem swerve = SwerveSubsystem.getInstance();
    IntakeArmSubsystem intakeArm = IntakeArmSubsystem.getInstance();
    ShooterArmSubsystem shooterArm = ShooterArmSubsystem.getInstance();
    ShooterSubsystem shooter = ShooterSubsystem.getInstance();

    public AutoContainer() {
        this.pathPlanner = PathPlannerHelper.getInstace();
        this.autoPaths = new HashMap<String, Command>();

        this.autoPaths.put("Drive FF Characterization", new FeedForwardCharacterization(swerve,
                swerve::runCharacterizationVolts,
                swerve::getCharacterizationVelocity));

        this.autoPaths.put("intake Arm FF Characterization", new FeedForwardCharacterization(intakeArm,
                intakeArm::runCharacterizationVolts,
                intakeArm::getCharacterizationVelocity));
                
        this.autoPaths.put("Shooter Arm FF Characterization", new FeedForwardCharacterization(shooterArm,
                shooterArm::runCharacterizationVolts,
                shooterArm::getCharacterizationVelocity));
                
        this.autoPaths.put("Upper Shooter FF Characterization", new FeedForwardCharacterization(shooter,
                shooter::UPRunCharacterizationVolts,
                shooter::getUPCharacterizationVelocity));
                
        this.autoPaths.put("Lower Shooter FF Characterization", new FeedForwardCharacterization(shooter,
                shooter::DownRunCharacterizationVolts,
                shooter::getDownCharacterizationVelocity));

        this.autoSelector = new CommandSelector(autoPaths, PrimoShuffleboard.getInstance().getCompTabTitle());
    }

    public Command getAutonomousCommand() {
        return this.autoSelector.getCommand();
    }
}
