// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.CommandGroupsFactory;
import frc.robot.a_robotCommandGroups.ReadyShootSpeaker;
import frc.robot.a_robotCommandGroups.ShootBase;
import frc.robot.a_robotCommandGroups.ShootSpeaker;
import frc.robot.a_robotCommandGroups.ShootStage;
import frc.robot.basicCommands.ShooterArmCommands.ZeroShooterArm;
import frc.robot.basicCommands.TakeFeedCommands.CollectUntilNote;
import frc.robot.basicCommands.TakeFeedCommands.TakeFeedJoystickSetSpeed;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.Telemetry;
import frc.robot.subsystems.swerve.TunerConstants;
import frc.robot.subsystems.takeFeed.TakeFeedSubsystem;

public class RobotContainer {

	/* Setting up bindings for necessary control of the swerve drive platform */
	public static final CommandXboxController driverJoystick = new CommandXboxController(0);
	public static final CommandXboxController operatorJoystick = new CommandXboxController(0);

	public final CommandSwerveDrivetrain swerve = TunerConstants.Swerve; // My drivetrain
	Telemetry logger = new Telemetry(TunerConstants.kSpeedAt12VoltsMps);

	private void configureBindings() {

		operatorJoystick.x().onTrue(new CollectUntilNote());
		operatorJoystick.y().onTrue(new ReadyShootSpeaker());
		operatorJoystick.b().onTrue(new InstantCommand(() -> ShooterSubsystem.getInstance().coast(),
				ShooterSubsystem.getInstance()));
		TakeFeedSubsystem.getInstance()
				.setDefaultCommand(new TakeFeedJoystickSetSpeed(() -> -operatorJoystick.getLeftY()));

		driverJoystick.y().onTrue(new InstantCommand(() -> swerve.seedFieldRelative()));
		driverJoystick.x().onTrue(new ShootBase());
		driverJoystick.rightBumper().whileTrue(new ShootSpeaker());
		driverJoystick.leftBumper().onTrue(CommandGroupsFactory.getSwitchDriveModeCommand());
		driverJoystick.povRight().onTrue(new ZeroShooterArm());
	}

	public RobotContainer() {
		swerve.setDefaultCommand(CommandGroupsFactory.getTeleopDriveCommand());
		swerve.registerTelemetry(logger::telemeterize);
		configureBindings();
		SmartDashboard.putString("swerve default command", swerve.getDefaultCommand().getName());
	}

	public Command getAutonomousCommand() {
		/* First put the drivetrain into auto run mode, then run the auto */
		return Commands.none();
	}
}
