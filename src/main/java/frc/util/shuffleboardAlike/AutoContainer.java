// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.util.shuffleboardAlike;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.a_robotCommandGroups.ShootBase;
import frc.robot.a_robotCommandGroups.ShootSpeaker;
import frc.robot.basicCommands.SwerveCommands.AutoAlignToSpeaker;
import frc.robot.basicCommands.TakeFeedCommands.CollectUntilNote;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.util.PathPlanner.PathPlannerHelper;

/** Add your docs here. */
public class AutoContainer {
    private CommandSelector autoSelector;
    private Map<String, Command> autoPaths;
    private PathPlannerHelper pathPlanner = PathPlannerHelper.getInstace();
    SwerveSubsystem swerve = SwerveSubsystem.getInstance();
    ShooterSubsystem shooter = ShooterSubsystem.getInstance();

    double waitForRot = 1;
    double headingAccuracy = 7;

    public AutoContainer() {
    Command baseTo2 = new SequentialCommandGroup(
            new ShootBase(),
            new ParallelCommandGroup(
                    pathPlanner.followChoreoPath("base to 2", false),
                    new CollectUntilNote()),
            new AutoAlignToSpeaker(),
            new ShootSpeaker());

    Command twoTo3 = new SequentialCommandGroup(
            new ParallelCommandGroup(
                    pathPlanner.followChoreoPath("2 to 1", false),
                    new CollectUntilNote()),
            new AutoAlignToSpeaker(),
            new ShootSpeaker());

    Command baseU1 = new SequentialCommandGroup(
            new AutoAlignToSpeaker(),
            new ShootSpeaker(),
            new ParallelCommandGroup(
                    pathPlanner.followChoreoPath("baseU to 1", false),
                    new CollectUntilNote()),
            new AutoAlignToSpeaker(),
            new ShootSpeaker());

    Command oneTo2 = new SequentialCommandGroup(
            new ParallelCommandGroup(
                    pathPlanner.followChoreoPath("1 to 2", false),
                    new CollectUntilNote()),
            new AutoAlignToSpeaker(),
            new ShootSpeaker());

    Command oneTo1m = new SequentialCommandGroup(
            new ParallelCommandGroup(
                    pathPlanner.followChoreoPath("1 to 1m", false),
                    new CollectUntilNote()),
            pathPlanner.followChoreoPath("1m shoot", false),
            new AutoAlignToSpeaker(),
            new ShootSpeaker());

    Command baseD3 = new SequentialCommandGroup(
            new AutoAlignToSpeaker(),
            new ShootSpeaker(),
            new ParallelCommandGroup(
                    pathPlanner.followChoreoPath("baseD to 3", false),
                    new CollectUntilNote()),
            new AutoAlignToSpeaker(),
            new ShootSpeaker());
    Command three5m = new SequentialCommandGroup(
            new ParallelCommandGroup(
                    pathPlanner.followChoreoPath("3 to 5m", false),
                    new CollectUntilNote()),
            pathPlanner.followChoreoPath("5m shoot", false),
            new AutoAlignToSpeaker(),
            new ShootSpeaker());

    Command baseD5m = new SequentialCommandGroup(
            new AutoAlignToSpeaker(),
            new ShootSpeaker(),
            new ParallelCommandGroup(
                    pathPlanner.followChoreoPath("down to 5m", false),
                    new CollectUntilNote()),
            pathPlanner.followChoreoPath("5m shoot", false),
            new AutoAlignToSpeaker(),
            new ShootSpeaker());

        this.autoPaths = new HashMap<String, Command>();

        this.autoPaths.put("no auto", Commands.none());

        this.autoPaths.put("base,2", new SequentialCommandGroup(
            new ShootBase(),
            new ParallelCommandGroup(
                    pathPlanner.followChoreoPath("base to 2", false),
                    new CollectUntilNote()),
            new AutoAlignToSpeaker(),
            new ShootSpeaker()));

        this.autoPaths.put("base,2,1", new SequentialCommandGroup(
                baseTo2,
                twoTo3
                ));

        this.autoPaths.put("baseU,1", baseU1);

        this.autoPaths.put("baseU,1,1m", new SequentialCommandGroup(
            baseU1,
            oneTo1m
        ));

        this.autoPaths.put("baseD,3", new SequentialCommandGroup(
            new AutoAlignToSpeaker(),
            new ShootSpeaker(),
            new ParallelCommandGroup(
                    pathPlanner.followChoreoPath("baseD to 3", false),
                    new CollectUntilNote()),
            new AutoAlignToSpeaker(),
            new ShootSpeaker()));

        this.autoPaths.put("baseD,3,5m", 
        new SequentialCommandGroup(
            baseD3,
            three5m
        ));

        this.autoPaths.put("baseD,5m", baseD5m);

        this.autoSelector = new CommandSelector(autoPaths, PrimoShuffleboard.getInstance().getCompTabTitle());
    }

    public Command getAutonomousCommand() {
        return this.autoSelector.getCommand();
    }
}
