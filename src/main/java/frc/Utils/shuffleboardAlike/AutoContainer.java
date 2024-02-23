// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.Utils.shuffleboardAlike;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.Utils.PathPlanner.PathPlannerHelper;
import frc.robot.aRobotOperations.AutoIntakeToFeeder;
import frc.robot.aRobotOperations.IntakeToFeeder;
import frc.robot.aRobotOperations.ShootSpeaker;
import frc.robot.aRobotOperations.ShootTouchingBase;
import frc.robot.basicCommands.IntakeArmCommands.IntakeArmDown;
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

    public AutoContainer() {
        this.autoPaths = new HashMap<String, Command>();
        this.autoPaths.put("base 2", pathPlanner.followPath("base to 2")
        .andThen(Commands.waitSeconds(0.2))
        .andThen(swerve.setHeadingCommand(new Rotation2d(Units.degreesToRadians(-90))))
        .until(()->Math.abs(swerve.getYaw().getDegrees() + 90)<3)
        .andThen(pathPlanner.followPath("2 to 3"))
        .andThen(swerve.disableHeadingCommand()));

        this.autoSelector = new CommandSelector(autoPaths, PrimoShuffleboard.getInstance().getCompTabTitle());
    }

    public Command getAutonomousCommand() {
        return this.autoSelector.getCommand();
    }
}
