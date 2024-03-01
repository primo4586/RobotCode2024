// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.utils.swerve.COTSFalconSwerveConstants;
import frc.utils.swerve.SwerveModuleConstants;

/** Add your docs here. */
public class SwerveConstants {

    public static final class swerveConstants {
        public static final double minimumErrorAligning = 0; 
        public static final PIDController aligningPID = new PIDController(0.12, 0, 0.0);

        public static final int pigeonID = 11;
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        public static final COTSFalconSwerveConstants chosenModule = COTSFalconSwerveConstants
                .SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L1);

        /* Drivetrain Constants */
        public static final double trackWidth = 0.62;
        public static final double wheelBase = 0.62;
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /*
         * Swerve Kinematics
         * No need to ever change this unless you are not doing a traditional
         * rectangular/square 4 module swerve
         */
        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = 8.14;
        public static final double angleGearRatio = 360 / ((150 / 7.0) / 1.0);

        /* Motor Inverts */
        public static final boolean angleMotorInvert = true;
        public static final InvertedValue driveMotorInvert = InvertedValue.CounterClockwise_Positive;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 35;
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 35;
        public static final int driveCurrentThreshold = 60;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /*
         * These values are used by the drive falcon to ramp in open loop and closed
         * loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
         */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;
        public static final double voltageComp = 12;

        /* Angle Motor PID Values */
        public static final double angleKP = 0.07;// chosenModule.angleKP;
        public static final double angleKI = 0.0;
        public static final double angleKD = 0.0;
        public static final double angleKFF = 0.0;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.5;
        public static final double XYSlowRatio = 0.25;
        public static final double rotationSlowRatio = 0.1;
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0;

        /* Neutral Modes */
        public static final IdleMode angleNeutralMode = IdleMode.kCoast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 1;
            public static final int canCoderID = 1;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.116455);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 2;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 2;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.474121);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 3;
            public static final int canCoderID = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.404541);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 4;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.389404);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }
    }

    public static final class AutoConstants { 
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 4.5;
        public static final double kMaxAngularSpeedRadiansPerSecond = 4.5;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = 4.5;

        public static final PIDConstants rotation_PID = new PIDConstants(3, 0);
        public static final PIDConstants XY_PID = new PIDConstants(5, 0);

        public static final double driveBaseRadius = Math.sqrt(Math.pow((swerveConstants.wheelBase / 2), 2)
                + Math.pow((swerveConstants.trackWidth / 2), 2));

        public static final ReplanningConfig replanningConfig = new ReplanningConfig(true, true);

        public static final PathConstraints pathConstraints = new PathConstraints(kMaxSpeedMetersPerSecond,
                kMaxAccelerationMetersPerSecondSquared, kMaxAngularSpeedRadiansPerSecond,
                kMaxAngularSpeedRadiansPerSecondSquared);

        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
}
