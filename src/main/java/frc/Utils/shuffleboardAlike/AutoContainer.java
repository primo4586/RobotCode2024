// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.Utils.shuffleboardAlike;

import static frc.robot.Constants.FeederConstants.getNoteSpeed;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.Utils.PathPlanner.PathPlannerHelper;
import frc.robot.aRobotOperations.CollectToFeeder;
import frc.robot.aRobotOperations.PrepareForShoot;
import frc.robot.aRobotOperations.ShootSpeaker;
import frc.robot.aRobotOperations.ShootTouchingBase;
import frc.robot.basicCommands.IntakeArmCommands.IntakeArmDown;
import frc.robot.basicCommands.IntakeCommands.IntakeSetSpeed;
import frc.robot.basicCommands.ShooterCommands.ShooterSetSpeed;
import frc.robot.basicCommands.feederCommands.FeedToShooter;
import frc.robot.basicCommands.feederCommands.FeedUntilNote;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.ShooterArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

/** Add your docs here. */
public class AutoContainer {
    private CommandSelector autoSelector;
    private Map<String, Command> autoPaths;
    private PathPlannerHelper pathPlanner = PathPlannerHelper.getInstace();
    SwerveSubsystem swerve = SwerveSubsystem.getInstance();
    IntakeArmSubsystem intakeArm = IntakeArmSubsystem.getInstance();
    ShooterArmSubsystem shooterArm = ShooterArmSubsystem.getInstance();
    ShooterSubsystem shooter = ShooterSubsystem.getInstance();

    double waitForRot = 1;
    double headingAccuracy = 7;

    public AutoContainer() {
        this.autoPaths = new HashMap<String, Command>();

        this.autoPaths.put("base,2", new SequentialCommandGroup(
                new ParallelCommandGroup(new ShootTouchingBase(), new IntakeArmDown()),
                new ParallelCommandGroup(pathPlanner.followChoreoPath("base to 2"), new FeedUntilNote(),
                        new IntakeSetSpeed(() -> getNoteSpeed)),
                new PrepareForShoot().until(()->shooter.getUpShooterSpeed()>90),
                new ShootSpeaker(),
                new ShooterSetSpeed(0)));

        this.autoPaths.put("baseU,1", new SequentialCommandGroup(
                new ParallelCommandGroup(new ShootSpeaker(), new IntakeArmDown()),
                swerve.setHeadingCommand(new Rotation2d(Units.degreesToRadians(22.5))),
                Commands.waitUntil(() -> Math.abs(Math.abs(swerve.getYaw().getDegrees()) - 22.5) < headingAccuracy),
                new ParallelCommandGroup(pathPlanner.followChoreoPath("baseU to 1"),
                        new FeedUntilNote(),
                        new IntakeSetSpeed(() -> getNoteSpeed)),
                new ShootSpeaker()));

        this.autoPaths.put("baseU,1,2", new SequentialCommandGroup(
                new ParallelCommandGroup(new ShootSpeaker(), new IntakeArmDown()),
                swerve.setHeadingCommand(new Rotation2d(Units.degreesToRadians(22.5))),
                Commands.waitUntil(() -> Math.abs(Math.abs(swerve.getYaw().getDegrees()) - 22.5) < headingAccuracy),
                new ParallelCommandGroup(pathPlanner.followChoreoPath("baseU to 1"),
                        new FeedUntilNote(),
                        new IntakeSetSpeed(() -> getNoteSpeed)),
                new ShootSpeaker(),
                swerve.setHeadingCommand(Rotation2d.fromDegrees(-90)),
                Commands.waitUntil(() -> Math.abs(Math.abs(swerve.getYaw().getDegrees()) + 90) < headingAccuracy),
                new ParallelCommandGroup(pathPlanner.followChoreoPath("1 to 2"), new FeedUntilNote(),
                        new IntakeSetSpeed(() -> getNoteSpeed)),
                swerve.setHeadingCommand(Rotation2d.fromDegrees(0)),
                new ShootSpeaker()));

        this.autoPaths.put("baseU,1,1m", new SequentialCommandGroup(
                new ParallelCommandGroup(new ShootSpeaker(), new IntakeArmDown()),
                swerve.setHeadingCommand(new Rotation2d(Units.degreesToRadians(22.5))),
                Commands.waitUntil(() -> Math.abs(Math.abs(swerve.getYaw().getDegrees()) - 22.5) < headingAccuracy),
                new ParallelCommandGroup(pathPlanner.followChoreoPath("baseU to 1"),
                        new FeedUntilNote(),
                        new IntakeSetSpeed(() -> getNoteSpeed)),
                new ShootSpeaker(),
                swerve.setHeadingCommand(new Rotation2d(Units.degreesToRadians(0))),
                Commands.waitUntil(() -> Math.abs(Math.abs(swerve.getYaw().getDegrees()) - 0) < headingAccuracy),
                new ParallelCommandGroup(pathPlanner.followChoreoPath("1 to 1m"),
                        new FeedUntilNote(),
                        new IntakeSetSpeed(() -> getNoteSpeed)),
                swerve.setHeadingCommand(Rotation2d.fromDegrees(22.5)),
                pathPlanner.followChoreoPath("1 to 1m"),
                new ShootSpeaker()));

        this.autoPaths.put("baseD,3", new SequentialCommandGroup(
                new ParallelCommandGroup(new ShootSpeaker(), new IntakeArmDown()),
                swerve.setHeadingCommand(new Rotation2d(Units.degreesToRadians(-22.5))),
                Commands.waitUntil(() -> Math.abs(Math.abs(swerve.getYaw().getDegrees()) + 22.5) < headingAccuracy),
                new ParallelCommandGroup(pathPlanner.followChoreoPath("baseD to 3"), new CollectToFeeder()),
                new ShootSpeaker()));

        this.autoSelector = new CommandSelector(autoPaths, PrimoShuffleboard.getInstance().getCompTabTitle());
    }

    public Command getAutonomousCommand() {
        return this.autoSelector.getCommand();
    }
}
