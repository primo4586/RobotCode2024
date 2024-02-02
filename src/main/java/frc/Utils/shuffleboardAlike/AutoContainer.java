// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.Utils.shuffleboardAlike;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import frc.Utils.PathPlanner.PathPlannerHelper;
import frc.robot.subsystems.SwerveSubsystem;

/** Add your docs here. */
public class AutoContainer {
    private CommandSelector autoSelector;
    private Map<String, Command> autoPaths;
    private PathPlannerHelper pathPlanner;
    SwerveSubsystem swerve = SwerveSubsystem.getInstance();

    public AutoContainer(){
        this.pathPlanner = PathPlannerHelper.getInstace();
        this.autoPaths = new HashMap<String, Command>();
        
        this.autoPaths.put("test2Meter", pathPlanner.followPath("test2Meter"));
        this.autoPaths.put("testSpin", pathPlanner.followPath("testSpin"));


        this.autoPaths.put("testSplineChoreo", pathPlanner.followChoreoPath("testSplineChoreo"));
        this.autoPaths.put("test2meterChoreo", pathPlanner.followChoreoPath("test2meterChoreo"));
        this.autoPaths.put("NewPath", pathPlanner.followChoreoPath("NewPath"));
        this.autoPaths.put("4piece", pathPlanner.followChoreoPath("4piece"));

        this.autoSelector = new CommandSelector(autoPaths, PrimoShuffleboard.getInstance().getCompTabTitle());
    }

    public Command getAutonomousCommand() {
        return this.autoSelector.getCommand();
    }
}
    