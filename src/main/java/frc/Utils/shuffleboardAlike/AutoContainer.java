// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.utils.shuffleboardAlike;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.utils.PathPlanner.PathPlannerHelper;

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
        this.autoPaths = new HashMap<String, Command>();

        // this.autoPaths.put("base,2", new SequentialCommandGroup(
        //         new ShootTouchingBase().until(()->shooter.getUpShooterSpeed()>60), 
        //         new FeederSetSpeedForever(1).withTimeout(0.5),
        //         new IntakeArmDown(),
        //         new ParallelCommandGroup(pathPlanner.followChoreoPath("base to 2", true), new FeedUntilNote(),
        //                 new IntakeSetSpeed(() -> getNoteSpeed)),
        //         new PrepareForShoot().repeatedly().until(()->shooter.getUpShooterSpeed()>65&&shooterArm.getArmPose()>20),
        //         new ParallelCommandGroup(new FeederSetSpeedForever(1),new PrepareForShoot().repeatedly()).withTimeout(2),
        //         new ShooterSetSpeed(0)));

        // this.autoPaths.put("baseU,1", new SequentialCommandGroup(
        //         // new AlignToAngle(AllianceFlipUtil.apply(Rotation2d.fromDegrees(0))),
        //         // pathPlanner.followChoreoPath("baseU out", true),
        //         new AutoAlignToSpeaker().until(()->swerve.headingPid.atSetpoint()),

        //         new PrepareForShoot().repeatedly().until(()->shooter.getUpShooterSpeed()>65),
        //         new ParallelCommandGroup(new FeederSetSpeedForever(1),new PrepareForShoot().repeatedly()).withTimeout(1),

        //         new AlignToAngle(AllianceFlipUtil.apply(Rotation2d.fromDegrees(0))),

        //         new IntakeArmDown(),
        //         new ParallelCommandGroup(pathPlanner.followChoreoPath("baseU to 1", true), new FeedUntilNote(),
        //                 new IntakeSetSpeed(() -> 0.45)),
        //         new IntakeSetSpeed(()->0),
        //         new IntakeArmUP(),        
        //         //check here
        //         new AutoAlignToSpeaker().withTimeout(1),
        //         new AutoAlignToSpeaker().until(()->swerve.headingPid.atSetpoint()),
        //         Commands.waitSeconds(0.1),
        //         new AutoAlignToSpeaker().until(()->swerve.headingPid.atSetpoint()),
        //         Commands.waitSeconds(0.1),
        //         new PrepareForShoot().repeatedly().until(()->shooter.getUpShooterSpeed()>65&&shooterArm.isArmReady()),
        //         new PrepareForShoot().repeatedly().until(()->shooter.getUpShooterSpeed()>65&&shooterArm.isArmReady()),
        //         new ParallelCommandGroup(new FeederSetSpeedForever(1),new PrepareForShoot().repeatedly()).withTimeout(1.5)
        // ));

        
        // this.autoPaths.put("baseU,1,2", new SequentialCommandGroup(
        //         // new AlignToAngle(AllianceFlipUtil.apply(Rotation2d.fromDegrees(0))),
        //         // pathPlanner.followChoreoPath("baseU out", true),
        //         new AutoAlignToSpeaker().until(()->swerve.headingPid.atSetpoint()),

        //         new PrepareForShoot().repeatedly().until(()->shooter.getUpShooterSpeed()>65),
        //         new ParallelCommandGroup(new FeederSetSpeedForever(1),new PrepareForShoot().repeatedly()).withTimeout(1),

        //         new AlignToAngle(AllianceFlipUtil.apply(Rotation2d.fromDegrees(0))),

        //         new IntakeArmDown(),
        //         new ParallelCommandGroup(pathPlanner.followChoreoPath("baseU to 1", true), new FeedUntilNote(),
        //                 new IntakeSetSpeed(() -> 0.45)),
        //         new IntakeSetSpeed(()->0),
        //         new IntakeArmUP(),        
        //         //check here
        //         new AutoAlignToSpeaker().withTimeout(1),
        //         new AutoAlignToSpeaker().until(()->swerve.headingPid.atSetpoint()),
        //         Commands.waitSeconds(0.1),
        //         new AutoAlignToSpeaker().until(()->swerve.headingPid.atSetpoint()),
        //         Commands.waitSeconds(0.1),
        //         new PrepareForShoot().repeatedly().until(()->shooter.getUpShooterSpeed()>65&&shooterArm.isArmReady()),
        //         new PrepareForShoot().repeatedly().until(()->shooter.getUpShooterSpeed()>65&&shooterArm.isArmReady()),
        //         new ParallelCommandGroup(new FeederSetSpeedForever(1),new PrepareForShoot().repeatedly()).withTimeout(1.5),

                
        //         new AlignToAngle(AllianceFlipUtil.apply(Rotation2d.fromDegrees(-90))),
        //         swerve.setHeadingCommand(Rotation2d.fromDegrees(-90)),
        //         new ParallelCommandGroup(pathPlanner.followChoreoPath("1 to 2", true), new FeedUntilNote(),
        //                 new IntakeSetSpeed(() -> 0.45)),
        //         new IntakeSetSpeed(()->0),    
        //         //check here
        //         new AutoAlignToSpeaker().withTimeout(0.5),
        //         new AutoAlignToSpeaker().until(()->swerve.headingPid.atSetpoint()),
        //         Commands.waitSeconds(0.1),
        //         new AutoAlignToSpeaker().until(()->swerve.headingPid.atSetpoint()),
        //         Commands.waitSeconds(0.1),
        //         new PrepareForShoot().repeatedly().until(()->shooter.getUpShooterSpeed()>65&&shooterArm.getArmPose()>15),
        //         new PrepareForShoot().repeatedly().until(()->shooter.getUpShooterSpeed()>65),
        //         new ParallelCommandGroup(new FeederSetSpeedForever(1),new PrepareForShoot().repeatedly()).withTimeout(0.5)
        // ));

        // this.autoPaths.put("baseU,1", new SequentialCommandGroup(
        //         new ParallelCommandGroup(new ShootSpeaker(), new IntakeArmDown()),
        //         swerve.setHeadingCommand(new Rotation2d(Units.degreesToRadians(22.5))),
        //         Commands.waitUntil(() -> Math.abs(Math.abs(swerve.getYaw().getDegrees()) - 22.5) < headingAccuracy),
        //         new ParallelCommandGroup(pathPlanner.followChoreoPath("baseU to 1"),
        //                 new FeedUntilNote(),
        //                 new IntakeSetSpeed(() -> getNoteSpeed)),
        //         new ShootSpeaker()));

        // this.autoPaths.put("baseU,1,2", new SequentialCommandGroup(
        //         new ParallelCommandGroup(new ShootSpeaker(), new IntakeArmDown()),
        //         swerve.setHeadingCommand(new Rotation2d(Units.degreesToRadians(22.5))),
        //         Commands.waitUntil(() -> Math.abs(Math.abs(swerve.getYaw().getDegrees()) - 22.5) < headingAccuracy),
        //         new ParallelCommandGroup(pathPlanner.followChoreoPath("baseU to 1"),
        //                 new FeedUntilNote(),
        //                 new IntakeSetSpeed(() -> getNoteSpeed)),
        //         new ShootSpeaker(),
        //         swerve.setHeadingCommand(Rotation2d.fromDegrees(-90)),
        //         Commands.waitUntil(() -> Math.abs(Math.abs(swerve.getYaw().getDegrees()) + 90) < headingAccuracy),
        //         new ParallelCommandGroup(pathPlanner.followChoreoPath("1 to 2"), new FeedUntilNote(),
        //                 new IntakeSetSpeed(() -> getNoteSpeed)),
        //         swerve.setHeadingCommand(Rotation2d.fromDegrees(0)),
        //         new ShootSpeaker()));

        // this.autoPaths.put("baseU,1,1m", new SequentialCommandGroup(
        //         new ParallelCommandGroup(new ShootSpeaker(), new IntakeArmDown()),
        //         swerve.setHeadingCommand(new Rotation2d(Units.degreesToRadians(22.5))),
        //         Commands.waitUntil(() -> Math.abs(Math.abs(swerve.getYaw().getDegrees()) - 22.5) < headingAccuracy),
        //         new ParallelCommandGroup(pathPlanner.followChoreoPath("baseU to 1"),
        //                 new FeedUntilNote(),
        //                 new IntakeSetSpeed(() -> getNoteSpeed)),
        //         new ShootSpeaker(),
        //         swerve.setHeadingCommand(new Rotation2d(Units.degreesToRadians(0))),
        //         Commands.waitUntil(() -> Math.abs(Math.abs(swerve.getYaw().getDegrees()) - 0) < headingAccuracy),
        //         new ParallelCommandGroup(pathPlanner.followChoreoPath("1 to 1m"),
        //                 new FeedUntilNote(),
        //                 new IntakeSetSpeed(() -> getNoteSpeed)),
        //         swerve.setHeadingCommand(Rotation2d.fromDegrees(22.5)),
        //         pathPlanner.followChoreoPath("1 to 1m"),
        //         new ShootSpeaker()));

        // this.autoPaths.put("baseD,3", new SequentialCommandGroup(
        //         new ParallelCommandGroup(new ShootSpeaker(), new IntakeArmDown()),
        //         swerve.setHeadingCommand(new Rotation2d(Units.degreesToRadians(-22.5))),
        //         Commands.waitUntil(() -> Math.abs(Math.abs(swerve.getYaw().getDegrees()) + 22.5) < headingAccuracy),
        //         new ParallelCommandGroup(pathPlanner.followChoreoPath("baseD to 3"), new CollectToFeeder()),
        //         new ShootSpeaker()));

        this.autoSelector = new CommandSelector(autoPaths, PrimoShuffleboard.getInstance().getCompTabTitle());
    }

    public Command getAutonomousCommand() {
        return this.autoSelector.getCommand();
    }
}
