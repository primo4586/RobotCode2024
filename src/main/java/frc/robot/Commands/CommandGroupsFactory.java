// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Misc;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.TunerConstants;

public class CommandGroupsFactory {
    private static final CommandXboxController driverJoystick = RobotContainer.driverJoystick;

    private static final CommandSwerveDrivetrain swerve = TunerConstants.Swerve; // My drivetrain

    static double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
    static double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

    public static SwerveRequest.FieldCentric teleopDrive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public static SwerveRequest.FieldCentricFacingAngle driveAlignedToSpeaker = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.Velocity);

    public static Command getSwitchDriveModeCommand() {
        return new InstantCommand(() -> {
            if (swerve.getDefaultCommand().getName() == getTeleopDriveCommand().getName()) {
                swerve.setDefaultCommand(getDriveAlignedToSpeakerCommand());
            } else {
                swerve.setDefaultCommand(getTeleopDriveCommand());
            }

        });
    }

    public static Command getTeleopDriveCommand() {

        Command teleopDriveCommand = swerve
                .applyRequest(() -> teleopDrive.withVelocityX(-driverJoystick.getLeftY() * MaxSpeed)
                        .withVelocityY(-driverJoystick.getLeftX() * MaxSpeed)
                        .withRotationalRate(-driverJoystick.getRightX() * MaxAngularRate))
                .ignoringDisable(true);
        
        teleopDriveCommand.setName("TeleopDriveCommand");

        return teleopDriveCommand;
    }

    public static Command getDriveAlignedToSpeakerCommand() {
        Translation2d speakerPose = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
                ? Misc.speakerPoseBlue
                : Misc.speakerPoseRed;

        Command driveAlignedCommand = swerve.applyRequest(() -> driveAlignedToSpeaker
                .withVelocityX(-driverJoystick.getLeftY() * MaxSpeed)
                .withVelocityY(-driverJoystick.getLeftX() * MaxSpeed)
                .withTargetDirection(swerve.getPose().getTranslation().minus(speakerPose).getAngle()))
                .ignoringDisable(true);
        
        driveAlignedCommand.setName("DriveAlignedToSpeakerCommand");

        return driveAlignedCommand;
    }

}
