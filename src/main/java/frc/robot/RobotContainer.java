// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooterArm.fuckGitHub;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.Telemetry;
import frc.robot.subsystems.swerve.TunerConstants;

public class RobotContainer {

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController driverJoystick = new CommandXboxController(0); // My joystick

  public final CommandSwerveDrivetrain swerve = TunerConstants.DriveTrain; // My drivetrain
  public final ShooterSubsystem shooter = ShooterSubsystem.getInstance();
  public final fuckGitHub shooterArmSubsystem = fuckGitHub.getInstance();
  public final IntakeSubsystem intake = IntakeSubsystem.getInstance();
  public final ClimbSubsystem climb = ClimbSubsystem.getInstance();

  /* Path follower */
  private Command runAuto = swerve.getAutoPath("Tests");

  // TODO: ShootBase
  // TODO: ShootStage
  // TODO: Ready Shoot Speaker
  // TODO: shooter default command
  // TODO: intake default command
  // TODO: sysid shooter
  // TODO: sysid shooter arm
  private void configureBindings() {
    DoubleSupplier distanceFromSpeaker = () -> swerve.getState().Pose.getTranslation().getDistance(null);
    
    //shoot speaker
    driverJoystick.rightBumper().whileTrue(new ParallelCommandGroup(
        new InstantCommand(()-> swerve.setDefaultCommand(driveAlignedToSpeakerCommand)),
        shooterArmSubsystem.speakerAngleEterapolateCommand(distanceFromSpeaker.getAsDouble()).repeatedly(),
        Commands.waitUntil(() -> (shooterArmSubsystem.isArmReady() && shooter.isMotorsAtVel()))
            .andThen(intake.feedShooterCommand())));

    driverJoystick.y().onTrue(swerve.runOnce(() -> swerve.seedFieldRelative()));

    SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    driverJoystick.a().whileTrue(swerve.applyRequest(() -> brake));

    driverJoystick.povRight().onTrue(shooterArmSubsystem.homeArmcCommand());

    driverJoystick.leftBumper().onTrue(new InstantCommand(() -> {
      if (swerve.getDefaultCommand().equals(teleopDriveCommand)) {
        swerve.setDefaultCommand(driveAlignedToSpeakerCommand);
      } else {
        swerve.setDefaultCommand(teleopDriveCommand);
      }
    }));

    /* Bindings for drivetrain characterization */
    /*
     * These bindings require multiple buttons pushed to swap between quastatic and
     * dynamic
     */
    /*
     * Back/Start select dynamic/quasistatic, Y/X select forward/reverse direction
     */
    driverJoystick.back().and(driverJoystick.y()).whileTrue(swerve.sysIdDynamic(Direction.kForward));
    driverJoystick.back().and(driverJoystick.x()).whileTrue(swerve.sysIdDynamic(Direction.kReverse));
    driverJoystick.start().and(driverJoystick.y()).whileTrue(swerve.sysIdQuasistatic(Direction.kForward));
    driverJoystick.start().and(driverJoystick.x()).whileTrue(swerve.sysIdQuasistatic(Direction.kReverse));

    swerve.setDefaultCommand(teleopDriveCommand);
    swerve.registerTelemetry(logger::telemeterize);
  }

  public RobotContainer() {
    configureBindings();
  }

  double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
  Telemetry logger = new Telemetry(MaxSpeed);

  SwerveRequest.FieldCentric teleopDrive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric driving in closed loop

  SwerveRequest.FieldCentricFacingAngle driveAlignedToSpeaker = new SwerveRequest.FieldCentricFacingAngle()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric driving in open loop

  Command driveAlignedToSpeakerCommand = swerve
      .applyRequest(() -> driveAlignedToSpeaker.withVelocityX(-driverJoystick.getLeftY() * MaxSpeed) // Drive forward
                                                                                                     // with negative
                                                                                                     // Y (forward)
          .withVelocityY(-driverJoystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
          .withTargetDirection(new Rotation2d()) // TODO: add target direction
      ).ignoringDisable(true);

  Command teleopDriveCommand = swerve
      .applyRequest(() -> teleopDrive.withVelocityX(-driverJoystick.getLeftY() * MaxSpeed) // Drive forward with
          // negative Y (forward)
          .withVelocityY(-driverJoystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
          .withRotationalRate(-driverJoystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X
      ).ignoringDisable(true);

  public Command getAutonomousCommand() {
    /* First put the drivetrain into auto run mode, then run the auto */
    return runAuto;
  }
}
