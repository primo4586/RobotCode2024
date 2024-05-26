// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooterArm.ShooterArmSubsystem;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.Telemetry;
import frc.robot.subsystems.swerve.TunerConstants;

public class RobotContainer {

	/* Setting up bindings for necessary control of the swerve drive platform */
	public static final CommandXboxController driverJoystick = new CommandXboxController(0);
	public static final CommandXboxController operatorJoystick = new CommandXboxController(0);

	public final CommandSwerveDrivetrain swerve = TunerConstants.Swerve; // My drivetrain
	private final ShooterSubsystem shooter = ShooterSubsystem.getInstance();
	private final ShooterArmSubsystem shooterArm = ShooterArmSubsystem.getInstance();
	private final IntakeSubsystem intake = IntakeSubsystem.getInstance();
	Telemetry logger = new Telemetry(TunerConstants.kSpeedAt12VoltsMps);

	/* Path follower */
	private Command runAuto = swerve.getAutoPath("New Auto");

	private void configureBindings() {
		// DRIVER
		// controls---------------------------------------------------------------------------------------

		// shoot speaker
		driverJoystick.rightBumper().whileTrue(CommandGroupsFactory.getShootSpeakerCommand());

		// shoot base
		driverJoystick.rightBumper().whileTrue(CommandGroupsFactory.getShootBaseCommand());

		// zero swerve rotation
		driverJoystick.y().onTrue(swerve.runOnce(() -> swerve.seedFieldRelative()));

		// brake
		SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
		driverJoystick.a().whileTrue(swerve.applyRequest(() -> brake));

		// home shooter arm
		driverJoystick.povRight().onTrue(shooterArm.homeArmcCommand());

		// switch swerve drive mode
		driverJoystick.leftBumper().onTrue(CommandGroupsFactory.getSwitchDriveModeCommand());

		// OPERATOR
		// controls---------------------------------------------------------------------------------------

		// Ready Shoot Speaker
		operatorJoystick.y().toggleOnTrue(CommandGroupsFactory.getReadyShootSpeakerCommand());

		// switch between manual intake and climb
		operatorJoystick.a().onTrue(CommandGroupsFactory.getSwitchIntakeClimbCommand());

		// intake until note has been detected
		operatorJoystick.b().onTrue(intake.intakeUntilNoteCommand());

		intake.setDefaultCommand(intake.setSpeedCommand(operatorJoystick.getLeftY()));
		shooter.setDefaultCommand(shooter.setShooterVel(ShooterConstants.IDLE_VELOCITY));
		shooterArm.setDefaultCommand(
				shooterArm.speakerAngleEterapolateCommand(Misc.distanceFromSpeaker.getAsDouble()));

		swerve.setDefaultCommand(CommandGroupsFactory.getTeleopDriveCommand());
		swerve.registerTelemetry(logger::telemeterize);
	}

	private void configureBindingsSysid() {

		// uncomment the corresponding subsystem to sysid

		// driverJoystick.back().and(driverJoystick.y()).whileTrue(swerve.sysIdDynamic(Direction.kForward));
		// driverJoystick.back().and(driverJoystick.x()).whileTrue(swerve.sysIdDynamic(Direction.kReverse));
		// driverJoystick.start().and(driverJoystick.y()).whileTrue(swerve.sysIdQuasistatic(Direction.kForward));
		// driverJoystick.start().and(driverJoystick.x()).whileTrue(swerve.sysIdQuasistatic(Direction.kReverse));

		// driverJoystick.back().and(driverJoystick.y()).whileTrue(shooter.sysIdDynamic(Direction.kForward));
		// driverJoystick.back().and(driverJoystick.x()).whileTrue(shooter.sysIdDynamic(Direction.kReverse));
		// driverJoystick.start().and(driverJoystick.y()).whileTrue(shooter.sysIdQuasistatic(Direction.kForward));
		// driverJoystick.start().and(driverJoystick.x()).whileTrue(shooter.sysIdQuasistatic(Direction.kReverse));

		// driverJoystick.back().and(driverJoystick.y()).whileTrue(shooterArm.sysIdDynamic(Direction.kForward));
		// driverJoystick.back().and(driverJoystick.x()).whileTrue(shooterArm.sysIdDynamic(Direction.kReverse));
		// driverJoystick.start().and(driverJoystick.y()).whileTrue(shooterArm.sysIdQuasistatic(Direction.kForward));
		// driverJoystick.start().and(driverJoystick.x()).whileTrue(shooterArm.sysIdQuasistatic(Direction.kReverse));

	}

	public RobotContainer() {
		NamedCommands.registerCommand("shoot speaker", CommandGroupsFactory.getShootSpeakerCommand());
		NamedCommands.registerCommand("shoot no align", AutoCommandFactory.getShootNoAlignCommand());
		NamedCommands.registerCommand("shoot base", CommandGroupsFactory.getShootBaseCommand());
		NamedCommands.registerCommand("intake", intake.intakeUntilNoteCommand().withTimeout(3));
		NamedCommands.registerCommand("missed note", AutoCommandFactory.driveToMidlineNoteForAuto());
		NamedCommands.registerCommand("shoot if has note", AutoCommandFactory.getShootIfHasNote());
		configureBindings();
		configureBindingsSysid();
	}

	public Command getAutonomousCommand() {
		/* First put the drivetrain into auto run mode, then run the auto */
		return runAuto;
	}
}
