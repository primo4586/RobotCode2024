// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.shuffleboardAlike;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooter.ShooterSubsystem;

/** Add your docs here. */
public class AutoContainer {
    private CommandSelector autoSelector;
    private Map<String, Command> autoPaths;
    ShooterSubsystem shooter = ShooterSubsystem.getInstance();

    double waitForRot = 1;
    double headingAccuracy = 7;

    public AutoContainer() {

        this.autoPaths = new HashMap<String, Command>();

        this.autoPaths.put("no auto", Commands.none());

        this.autoSelector = new CommandSelector(autoPaths, PrimoShuffleboard.getInstance().getCompTabTitle());
    }

    public Command getAutonomousCommand() {
        return this.autoSelector.getCommand();
    }
}
