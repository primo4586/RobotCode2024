// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
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
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooterArm.ShooterArmSubsystem;
import frc.robot.subsystems.shooterArm.ShooterArmConstants;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.TunerConstants;

public class CommandGroupsFactory {
    private final static CommandXboxController driverJoystick = RobotContainer.driverJoystick;
    private final static CommandXboxController operatorJoystick = RobotContainer.operatorJoystick;

    private static final CommandSwerveDrivetrain swerve = TunerConstants.Swerve; // My drivetrain
    private static final ShooterSubsystem shooter = ShooterSubsystem.getInstance();
    private static final ShooterArmSubsystem shooterArm = ShooterArmSubsystem.getInstance();
    private static final IntakeSubsystem intake = IntakeSubsystem.getInstance();
    private static final ClimbSubsystem climb = ClimbSubsystem.getInstance();
    private static Translation2d speakerPoseBlue = new Translation2d(Units.inchesToMeters(-1.5),
            Units.inchesToMeters(218.42));
    private static Translation2d speakerPoseRed = new Translation2d(Units.inchesToMeters(-1.5),
            Units.inchesToMeters(652.73));
    private static DoubleSupplier distanceFromSpeaker = () -> swerve.getState().Pose.getTranslation().getDistance(
            DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? speakerPoseBlue
                    : speakerPoseRed);

    private static boolean climbing = false;// are we climbing or intaking

    public static Command getShootSpeakerCommand() {
        return new ParallelDeadlineGroup(
                Commands.waitSeconds(0.02).andThen(// wait 1 rio cycle,
                        // wait until ready too shoot,
                        Commands.waitUntil(() -> (shooterArm.isArmReady()
                                && shooter.isMotorsAtVel()))
                                .andThen(intake.feedShooterCommand())), // finally shoot

                new InstantCommand(() -> swerve.setDefaultCommand(getDriveAlignedToSpeakerCommand())),
                shooterArm.speakerAngleEterapolateCommand(distanceFromSpeaker.getAsDouble())
                        .repeatedly(), // repeatedly so it will update if we moved
                shooter.setSpeakerVel());
    }

    public static Command getShootBaseCommand() {
        return new ParallelDeadlineGroup(
                Commands.waitSeconds(0.02).andThen(// wait 1 rio cycle,
                        // wait until ready too shoot,
                        Commands.waitUntil(() -> (shooterArm.isArmReady()
                                && shooter.isMotorsAtVel()))
                                .andThen(intake.feedShooterCommand())), // finally shoot
                shooterArm.moveArmToCommand(ShooterArmConstants.SHOOT_BASE_ANGLE).repeatedly(),
                shooter.setSpeakerVel().repeatedly());
    }

    public static Command getSwitchDriveModeCommand() {
        return new InstantCommand(() -> {
            if (swerve.getDefaultCommand().getName() == getTeleopDriveCommand().getName()) {
                swerve.setDefaultCommand(getDriveAlignedToSpeakerCommand());
            } else {
                swerve.setDefaultCommand(getTeleopDriveCommand());
            }

        });
    }

    public static Command getReadyShootSpeakerCommand() {
        return new ParallelCommandGroup(
                shooterArm.speakerAngleEterapolateCommand(distanceFromSpeaker.getAsDouble())
                        .repeatedly(),
                Commands.waitUntil(() -> (shooterArm.isArmReady() && shooter.isMotorsAtVel()))
                        .andThen(intake.feedShooterCommand()))
                .repeatedly();
    }

    public static Command getSwitchIntakeClimbCommand() {
        return new InstantCommand(() -> {
            if (climbing) {
                climb.removeDefaultCommand();
                intake.setDefaultCommand(intake.setSpeedCommand(operatorJoystick.getLeftY()));
                climbing = false;
            } else {
                climb.setDefaultCommand(climb.moveClimb(() -> operatorJoystick.getLeftY(),
                        () -> operatorJoystick.getRightY()));
                intake.removeDefaultCommand();
                climbing = true;
            }
        });
    }

    public static Command getTeleopDriveCommand() {
        return swerve
                .applyRequest(() -> teleopDrive.withVelocityX(-driverJoystick.getLeftY() * MaxSpeed)
                        .withVelocityY(-driverJoystick.getLeftX() * MaxSpeed)
                        .withRotationalRate(-driverJoystick.getRightX() * MaxAngularRate))
                .ignoringDisable(true);
    }

    public static Command getDriveAlignedToSpeakerCommand() {
        return swerve
                .applyRequest(() -> driveAlignedToSpeaker
                        .withVelocityX(-driverJoystick.getLeftY() * MaxSpeed)
                        .withVelocityY(-driverJoystick.getLeftX() * MaxSpeed))
                .ignoringDisable(true);
    }

    static double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
    static double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

    public static SwerveRequest.FieldCentric teleopDrive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric driving in
                                                                     // closed loop

    public static SwerveRequest.FieldCentricFacingAngle driveAlignedToSpeaker = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric driving in open
                                                                     // loop
}