// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooterArm.ShooterArmSubsystem;
import frc.robot.subsystems.shooterArm.shooterArmConstants;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.Telemetry;
import frc.robot.subsystems.swerve.TunerConstants;

public class RobotContainer {

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController driverJoystick = new CommandXboxController(0);
  private final CommandXboxController operatorJoystick = new CommandXboxController(0);

  public final CommandSwerveDrivetrain swerve = TunerConstants.Swerve; // My drivetrain
  private final ShooterSubsystem shooter = ShooterSubsystem.getInstance();
  private final ShooterArmSubsystem shooterArm = ShooterArmSubsystem.getInstance();
  private final IntakeSubsystem intake = IntakeSubsystem.getInstance();
  private final ClimbSubsystem climb = ClimbSubsystem.getInstance();

  private boolean climbing = false;// are we climbing or intaking

  /* Path follower */
  private Command runAuto = swerve.getAutoPath("Tests");

  private void configureBindings() {
    Translation2d speakerPoseBlue = new Translation2d(Units.inchesToMeters(-1.5), Units.inchesToMeters(218.42));
    Translation2d speakerPoseRed = new Translation2d(Units.inchesToMeters(-1.5), Units.inchesToMeters(652.73));
    DoubleSupplier distanceFromSpeaker = () -> swerve.getState().Pose.getTranslation().getDistance(
        DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? speakerPoseBlue : speakerPoseRed);

    // DRIVER
    // controls---------------------------------------------------------------------------------------

    // shoot speaker
    driverJoystick.rightBumper().whileTrue(new ParallelDeadlineGroup(
        Commands.waitSeconds(0.02).andThen(// wait 1 rio cycle,
            Commands.waitUntil(() -> (shooterArm.isArmReady() && shooter.isMotorsAtVel()))// wait until ready too shoot,
                .andThen(intake.feedShooterCommand())), // finally shoot

        new InstantCommand(() -> swerve.setDefaultCommand(driveAlignedToSpeakerCommand)),
        shooterArm.speakerAngleEterapolateCommand(distanceFromSpeaker.getAsDouble()).repeatedly(), // repeatedly so it
                                                                                                   // will check if we
                                                                                                   // moved
        shooter.setSpeakerVel()));

    // shoot speaker
    driverJoystick.rightBumper().whileTrue(new ParallelDeadlineGroup(
        Commands.waitSeconds(0.02).andThen(// wait 1 rio cycle,
            Commands.waitUntil(() -> (shooterArm.isArmReady() && shooter.isMotorsAtVel()))// wait until ready too shoot,
                .andThen(intake.feedShooterCommand())), // finally shoot

        shooterArm.moveArmToCommand(shooterArmConstants.SHOOT_BASE_ANGLE).repeatedly(),
        shooter.setSpeakerVel().repeatedly()));

    // zero swerve rotation
    driverJoystick.y().onTrue(swerve.runOnce(() -> swerve.seedFieldRelative()));

    // brake
    SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    driverJoystick.a().whileTrue(swerve.applyRequest(() -> brake));

    // home shooter arm
    driverJoystick.povRight().onTrue(shooterArm.homeArmcCommand());

    // switch swerve drive mode
    driverJoystick.leftBumper().onTrue(new InstantCommand(() -> {
      if (swerve.getDefaultCommand().equals(teleopDriveCommand)) {
        swerve.setDefaultCommand(driveAlignedToSpeakerCommand);
      } else {
        swerve.setDefaultCommand(teleopDriveCommand);
      }

    }));

    // OPERATOR
    // controls---------------------------------------------------------------------------------------

    // Ready Shoot Speaker
    operatorJoystick.y().toggleOnTrue(new ParallelCommandGroup(
        shooterArm.speakerAngleEterapolateCommand(distanceFromSpeaker.getAsDouble()).repeatedly(),
        Commands.waitUntil(() -> (shooterArm.isArmReady() && shooter.isMotorsAtVel()))
            .andThen(intake.feedShooterCommand()))
        .repeatedly());

    // switch between manual intake and climb
    operatorJoystick.a().onTrue(new InstantCommand(() -> {
      if (climbing) {
        climb.removeDefaultCommand();
        intake.setDefaultCommand(intake.setSpeedCommand(operatorJoystick.getLeftY()));
        climbing = false;
      } else {
        climb.setDefaultCommand(climb.moveClimb(() -> operatorJoystick.getLeftY(), () -> operatorJoystick.getRightY()));
        intake.removeDefaultCommand();
        climbing = true;
      }
    }));

    operatorJoystick.b().onTrue(intake.intakeuntilNoteCommand());

    intake.setDefaultCommand(intake.setSpeedCommand(operatorJoystick.getLeftY()));
    shooter.setDefaultCommand(shooter.setShooterVel(ShooterConstants.IDLE_VELOCITY));

    swerve.setDefaultCommand(teleopDriveCommand);
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
    configureBindings();
    configureBindingsSysid();
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
