// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Misc;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooterArm.ShooterArmSubsystem;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.TunerConstants;
import frc.robot.subsystems.vision.ObjectDetectionCamera;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;

public class AutoCommandFactory {

    private static final CommandSwerveDrivetrain swerve = TunerConstants.Swerve; // My drivetrain
    private static final ShooterSubsystem shooter = ShooterSubsystem.getInstance();
    private static final ShooterArmSubsystem shooterArm = ShooterArmSubsystem.getInstance();
    private static final IntakeSubsystem intake = IntakeSubsystem.getInstance();

    private static final ObjectDetectionCamera objectDetectionCamera = Misc.objectDetectionCamera;

    private static SwerveRequest.FieldCentricFacingAngle driveAlignedAngle = new SwerveRequest.FieldCentricFacingAngle()
            .withDriveRequestType(DriveRequestType.Velocity);

    private static SwerveRequest.RobotCentric fieldCentric = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.Velocity);

    /**
     * Command that try to collect the closest note in the midline after we were
     * stolen from
     */
    public static Command driveToMidlineNoteForAuto() {
        if (intake.getHasNote())
            return Commands.none();

        Pose2d startPose = swerve.getPose();
        if (startPose.getY() > 4.105) {// 4.105 meters is half field height
            driveAlignedAngle.TargetDirection = Rotation2d.fromDegrees(90);
            driveAlignedAngle.VelocityY = -2;
        } else {
            driveAlignedAngle.TargetDirection = Rotation2d.fromDegrees(270);
            driveAlignedAngle.VelocityY = 2;
        }

        Command driveToNote = swerve
                .applyRequest(() -> driveAlignedAngle.withVelocityX(TunerConstants.TRANSLATION_PID.calculate(
                        swerve.getPose().getX(),
                        16.54 / 2))// 16.54 meters is field width
                ).until(() -> objectDetectionCamera.getDetectingObject())
                .andThen(swerve.applyRequest(() -> fieldCentric
                        .withVelocityX(objectDetectionCamera.getAngleFromTarget() < 10 ? 2 : 0)
                        .withRotationalRate(
                                TunerConstants.ROTATION_PID.calculate(objectDetectionCamera.getAngleFromTarget(), 0))));

        return new ParallelDeadlineGroup(intake.intakeUntilNoteCommand(), driveToNote)
                .andThen(swerve.applyRequest(() -> new SwerveRequest.SwerveDriveBrake()));
    }

    /**
     * Command to shoot with no alignment.
     */
    private static Command shootNoAlignCommand() {// TODO: handle out of range
        return Commands.waitUntil(Misc.isWithinShootingRange).andThen(
                new ParallelDeadlineGroup(
                        Commands.waitSeconds(0.02).andThen(// wait 1 rio cycle,
                                // wait until ready too shoot,
                                Commands.waitUntil(() -> (shooterArm.isArmReady()
                                        && shooter.isMotorsAtVel()))
                                        .andThen(intake.feedShooterCommand())), // finally shoot

                        shooterArm.speakerAngleEterapolateCommand(Misc.distanceFromSpeaker.getAsDouble()),
                        shooter.setSpeakerVel()))
                .withTimeout(1.5);
    }

    public static Command getShootNoAlignCommand() {// TODO: handle out of range
        if (Misc.isWithinShootingRange.getAsBoolean()) {
            return shootNoAlignCommand().until(() -> !Misc.isWithinShootingRange.getAsBoolean())
                    .andThen(CommandGroupsFactory.getYeetCommand());
        }

        return shootNoAlignCommand();
    }

    public static Command getShootIfHasNote() {
        if (intake.getHasNote())
            return CommandGroupsFactory.getShootSpeakerCommand();

        return CommandGroupsFactory.getYeetCommand();
    }
}
