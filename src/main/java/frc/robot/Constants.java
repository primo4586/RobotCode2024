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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.Utils.interpolation.InterpolationMap;
import frc.Utils.swerve.COTSFalconSwerveConstants;
import frc.Utils.swerve.SwerveModuleConstants;
import frc.Utils.swerve.COTSFalconSwerveConstants.driveGearRatios;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public final class Constants {

    public static final double stickDeadband = 0.1;
    
    public static final String canBus_name = "canBus";

    public static final class IntakeArmConstants { // TODO: This must be tuned to specific robot
        // mm
        public static final int IntakeArmMotorID = 8;
        public static final double mmVelocity = 100;
        public static final double mmAcceleration = 800;
        public static final double mmJerk = 6000;
        public static final double KP = 0.0;
        public static final double KD = 0.0;
        public static final double KS = 0.051238;
        public static final double KV = 0.11136;
        public static final double KA = 0.0026838;


        public static final double PeakForwardVoltage = 12;
        public static final double PeakReverseVoltage = -12;
        public static final int encoderCountsPerRevolution = 1;
        public static final double gearRatio = 1;
        public static final double TICKS_PER_DEGREE = encoderCountsPerRevolution * gearRatio / 360.0;
        public static final boolean ForwardSoftLimitEnable = true;
        public static final double ForwardSoftLimitThreshold = 10;
        public static final boolean ReverseSoftLimitEnable = true;
        public static final double RevesrseSoftLimitThreshold = -170;
        // not mm
        public static final double minimumError = 0;
        public static final double intakeArmStartingValue = 0;
        public static final double zeroEncoderValue = 0;
        public static final double intakeArmZeroSpeed = 0.3;
        public static final double intakeArmUpSpeed = 1;
        public static final double intakeArmDownSpeed = -1;
        // switch
        public static final int intakeArmUpSwitchID = 0;
        public static final int intakeArmDownSwitchID = 3;

        // set points
        public static final double AmpSetPoint = 10;
        public static final double SafeSetPoint = 0;
        public static final double intakeSetPoint = -160;
        

    }

    public static final class Swerve {
        public static final double minimumErrorAligning = 0; // TODO: This must be tuned to specific robot
        public static final PIDController aligningPID = new PIDController(0.1, 0, 0.0);

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
        public static final double maxSpeed = 4.5; // TODO: This must be tuned to specific robot
        public static final double XYSlowRatio = 0.1; // TODO: make it more accrute
        public static final double rotationSlowRatio = 0.1; // TODO: make it more accrute
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0; // TODO: This must be tuned to specific robot

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

    public static final class AutoConstants { // TODO: The below constants are used in the example auto, and must be
                                              // tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 4.5;
        public static final double kMaxAngularSpeedRadiansPerSecond = 4.5;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = 4.5;

        public static final PIDConstants rotation_PID = new PIDConstants(3, 0);
        public static final PIDConstants XY_PID = new PIDConstants(5, 0);

        public static final double driveBaseRadius = Math.sqrt(Math.pow((Constants.Swerve.wheelBase / 2), 2)
                + Math.pow((Constants.Swerve.trackWidth / 2), 2));

        public static final ReplanningConfig replanningConfig = new ReplanningConfig(true, true);

        public static final PathConstraints pathConstraints = new PathConstraints(kMaxSpeedMetersPerSecond,
                kMaxAccelerationMetersPerSecondSquared, kMaxAngularSpeedRadiansPerSecond,
                kMaxAngularSpeedRadiansPerSecondSquared);

        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static class Vision {
        public static final String kRightCameraName = "right_Camera";
        public static final String kLeftCameraName = "left_Camera";
        // Cam mounted facing forward, half a meter forward of center, half a meter up
        // from center.
        public static final Transform3d kRightRobotToCam = new Transform3d(new Translation3d(-0.339, 0.14, 0.5),
                new Rotation3d(Math.asin(30.6/55.3), 0, Units.degreesToRadians(180+16)));
        public static final Transform3d kLeftRobotToCam = new Transform3d(new Translation3d(-0.339, 0.14, 0.5),
                new Rotation3d(Math.asin(30.6/55.3), 0, -Units.degreesToRadians(180+14.24)));

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

    public static class ShooterArmConstants {

        // technical Constants
        public static final int ShooterArmID = 5;
        public static final int SwitchID = 2;
        public static final int encoderCountsPerRevolution = 1;
        public static final double gearRatio = 100.0/22.0 * 100.0;
        public static final double TICKS_PER_DEGREE = encoderCountsPerRevolution * gearRatio / 360.0;

        // condition Costants
        public static final double minimumError = 2.0;

        // motionMagic Constants
        public static final double mmCruise = 80;
        public static final double mmAcceleration = 300;
        public static final double mmJerk = 1600;

        public static final double kp = 0.1;
        public static final double kd = 0.0;
        public static final double ks = 0.032658;
        public static final double ka = 0.001121;
        public static final double kv = 0.13707;

        // MaxVol Constant
        public static final double peekReverseVoltage = -11.5;
        public static final double peekForwardVoltage = 11.5;

        // Constant limit values
        public static final double forwardLimit = 300;
        public static final double backwordLimit = 300;

        // ArmPoseReset Constant
        public static final double resetPose = 0.0;
        public static final double resetSpeed = -0.1;

        public static final double shooterArmStartPose = 00;

        public static final InterpolationMap SHOOTER_ANGLE_INTERPOLATION_MAP = new InterpolationMap()
                .put(2, 5.7)        
                .put(2.124, 12)
                .put(2.3, 10.55)
                .put(2.577, 15)
                .put(2.6, 15.4)
                .put(2.647, 16.125)
                .put(2.9, 20.2)
                .put(3, 21.79)
                .put(3.1, 23.4)
                .put(3.2, 25)
                .put(3.4, 28.2)
                .put(3.45, 29)
                .put(3.7, 33.02)
                .put(3.915, 36.474)
                .put(4, 37.84)
                .put(4.2, 4.1)
                .put(4.3, 42.7)
                .put(4.4, 44.3)
                .put(4.6, 47.5)
                .put(4.8, 50.7)
                .put(4.9, 52.3)
                .put(5, 56.9);

        public static final double ShootBaseAngle = 0; //6.1064453125
        public static final double ShootStageAngle = 3;
    }

    public static class ShooterConstants {
        // m_shooterMotor ID
        public static final int kUpMotorShooterID = 6;
        public static final int kDownMotorShooterID = 7;

        // Motion Magic Values
        public static final int MotionMagicCruiseVelocity = 80;
        public static final int MotionMagicAcceleration = 160;
        public static final int MotionMagicJerk = 1600;

        public static final double PeakForwardVoltage = 11.5;
        public static final double PeakReverseVoltage = -11.5;

        public static final double GearRatio = 0.5;

        public static final int MaxError = 5;

        // PID values
        public static final double upKP = 0.06;//2
        public static final double upKD = 0.0;
        public static final double upKS = 0.16 ;
        public static final double upKV = 0.057;
        public static final double upKA = 0.0;


        public static final double downKP = 0.02;
        public static final double downKD = 0.0;
        public static final double downKS = 0.2998046875;
        public static final double downKV = 0.058;
        public static final double downKA = 0.079175;


        // Interpolation Map
        public static final InterpolationMap ShooterInterpolation = new InterpolationMap()
                .put(2.577, 100)
                .put(2.124, 100)
                .put(3.2, 100);

        public static final double ShootBaseSpeed = 80;
        public static final double ShootStageSpeed = 3;
    }

    public static class IntakeConstants {
        public static final int intakeNoteSensorID = 5;//todo
        public static final int IntakeMotorID = 7;
        public static final double getNoteSpeed = 0.9;
        public static final double feedToTrapSpeed = 0.4;
    }

    public static class FeederConstants {

        public static final int FeederShootSpeed = 1;
        public static final int FeederMotorId = 9;
        public static final int feederNoteSensorID = 4;//todo
        public static final double FeederMotorSpeed = 0.8;
        public static final double getNoteSpeed = 0.9;
        public static final double TimeToFeed = 0.7;

    }

    public static class TrapArmConstants {
        public static final int ArmMotorID = 10;
        public static final int OuterSwitchID = 9;//todo
        public static final double GoInSpeed = -0.1;
        public static final double GoOutSpeed = 0.1;

    }

    public static class trapConstants {
        public static final int TRAP_MOTOR_ID = 11;
        public static final int TrapNoteSensorID = 0;//todo
        public static final double TrapCollectSpeed = 0.1;
    }

    public static class climbingConstants {
        public static final int M_CLIMBINGRIGHT_MOTOR_ID = 6;
        public static final int M_CLIMBINGLEFT_MOTOR_ID = 5;
    }
}
