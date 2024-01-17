package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.Utils.interpolation.InterpolationMap;
import frc.Utils.swerve.COTSFalconSwerveConstants;
import frc.Utils.swerve.SwerveModuleConstants;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public final class Constants {
    public static final double stickDeadband = 0.1;

    public static final class Swerve {
        public static final int pigeonID = 10;
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        public static final COTSFalconSwerveConstants chosenModule =
            COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L1);

        /* Drivetrain Constants */
        public static final double trackWidth = 0.62;
        public static final double wheelBase = 0.62;
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
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

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;
        public static final double voltageComp = 12;

        /* Angle Motor PID Values */
        public static final double angleKP = 0.08;//chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;
        public static final double angleKFF = chosenModule.angleKD;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.2; //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0.15; //TODO: This must be tuned to specific robot
        public static final double driveKV = 2.7;
        public static final double driveKA = 0;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.5; //TODO: This must be tuned to specific robot
        public static final double XYSlowRatio = 0.25; // TODO: make it more accrute 
        public static final double rotationSlowRatio = 0.25; // TODO: make it more accrute 
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0; //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final IdleMode angleNeutralMode = IdleMode.kCoast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 6;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 2;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.113770 );
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 1;
            public static final int canCoderID = 1;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.474121);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 3;
            public static final int canCoderID = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.403320);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { 
            public static final int driveMotorID = 8;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 4;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.387695 );
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 3.5;
        public static final double kMaxAccelerationMetersPerSecondSquared = 9;
        public static final double kMaxAngularSpeedRadiansPerSecond = 9;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = 9;

        public static final PIDConstants rotation_PID = new PIDConstants(3, 0);
        public static final PIDConstants XY_PID = new PIDConstants(3, 0);

        public static final double driveBaseRadius = 
                        Math.sqrt(Math.pow((Constants.Swerve.wheelBase / 2), 2)
                        + Math.pow((Constants.Swerve.trackWidth / 2), 2));

        public static final ReplanningConfig replanningConfig = 
                new ReplanningConfig(true, true,
                        0.5, 1);
        
        public static final PathConstraints pathConstraints = new PathConstraints(kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared, kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
                        
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static class Vision {
    public static final String kRightCameraName = "Arducam_OV9281_USB_Camera";
    public static final String kLeftCameraName = "YOUR CAMERA NAME";
    // Cam mounted facing forward, half a meter forward of center, half a meter up
    // from center.
    public static final Transform3d kRightRobotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5),
        new Rotation3d(0, 0, 0));
    public static final Transform3d kLeftRobotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5),
        new Rotation3d(0, 0, 0));

    // The layout of the AprilTags on the field
    public static final AprilTagFieldLayout kTagLayout = AprilTagFields.kDefaultField.loadAprilTagLayoutField(); 

    // The standard deviations of our vision estimated poses, which affect
    // correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> kRightSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kRightMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

    public static final Matrix<N3, N1> kLeftSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kLeftMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

    public static final Pose2d target = new Pose2d(1, 1, new Rotation2d(Units.degreesToRadians(0)));
  }

    public static class ShooterConstants {
        //m_shooterMotor ID
        public static final int kMotorShooterID = 20;

        //Motion Magic Values
        public static final int MotionMagicCruiseVelocity = 5;
        public static final int MotionMagicAcceleration = 10;
        public static final int MotionMagicJerk = 50;

        public static final double PeakForwardVoltage = 11.5;
        public static final double PeakReverseVoltage = -11.5;

        public static final int SensorToMechanismRatio = 50;

        public static final int MaxError = 13;
        
        //PID values
        public static final int kP = 24;
        public static final double kD = 0.1;
        public static final double kS = 0.12;
        public static final double kV = 0.25;

        //Interpolation Map
        public static final InterpolationMap ShooterInterpolation = new InterpolationMap()
        .put(1,9)
        .put(1.2, 9.2)
        .put(1.4, 9.4)
        .put(1.6, 9.6)
        .put(1.8, 9.8)
        .put(2, 10)
        .put(2.1, 10.2);

    }
}
