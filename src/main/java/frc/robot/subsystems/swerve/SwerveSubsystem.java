package frc.robot.subsystems.swerve;

import frc.robot.FieldConstants;
import frc.robot.subsystems.swerve.SwerveConstants.swerveConstants;
import frc.robot.subsystems.swerve.SwerveConstants.swerveConstants.Mod0;
import frc.robot.subsystems.swerve.SwerveConstants.swerveConstants.Mod1;
import frc.robot.subsystems.swerve.SwerveConstants.swerveConstants.Mod2;
import frc.robot.subsystems.swerve.SwerveConstants.swerveConstants.Mod3;
import frc.utils.vision.Vision;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Represents the swerve driver subsystem
 * @implNote This is a singleton
 * 
 */
public class SwerveSubsystem extends SubsystemBase {
    public SwerveDrivePoseEstimator poseEstimation;
    public SwerveDriveOdometry odometry;
    public SwerveModule[] mSwerveMods;
    public TalonSRX talonSRX;
    public PigeonIMU gyro;
    public Vision vision;
    Field2d field2d = new Field2d();
    Rotation2d simYaw = new Rotation2d();
    public Rotation2d headingSupplier = null;
    public PIDController headingPid = swerveConstants.aligningPID;

    private static SwerveSubsystem instance = new SwerveSubsystem();

    // singelton
    public static SwerveSubsystem getInstance() {
        if (instance == null) {
            instance = new SwerveSubsystem();
        }
        return instance;
    }

    private SwerveSubsystem() {
        talonSRX = new TalonSRX(swerveConstants.pigeonID);
        gyro = new PigeonIMU(talonSRX);
        gyro.configFactoryDefault();
        zeroGyro();

        SmartDashboard.putData("field", field2d);

        vision = Vision.getInstance();

        mSwerveMods = new SwerveModule[] {
                new SwerveModule(0, Mod0.constants),
                new SwerveModule(1, Mod1.constants),
                new SwerveModule(2, Mod2.constants),
                new SwerveModule(3, Mod3.constants)
        };

            poseEstimation = new SwerveDrivePoseEstimator(swerveConstants.swerveKinematics, getYaw(), getModulePositions(),
                    new Pose2d(0, 0, new Rotation2d(0)));
        
        odometry = new SwerveDriveOdometry(swerveConstants.swerveKinematics, getYaw(), getModulePositions());
                
        headingPid.enableContinuousInput(-180, 180);
        headingPid.setTolerance( 1);
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop,
            boolean scaled) {
        SwerveModuleState[] swerveModuleStates = swerveConstants.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        headingSupplier == null ? rotation
                                : headingPid.calculate(getPose().getRotation().getDegrees(),
                                        headingSupplier.getDegrees()),
                        getYaw())
                        : new ChassisSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, swerveConstants.maxSpeed);

        if (!scaled) {
            for (SwerveModule mod : mSwerveMods) {
                mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
            }
        } else {
            for (SwerveModule mod : mSwerveMods) {
                mod.scaledSetDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
            }
        }

    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, swerveConstants.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        setModuleStates(swerveConstants.swerveKinematics.toSwerveModuleStates(chassisSpeeds));
    }

    public Pose2d getPose() {
        return poseEstimation.getEstimatedPosition();
    }

    public Pose2d getOdometry() {
        return odometry.getPoseMeters();
    }

    public Command resetOdometry(Pose2d pose) {
        return runOnce(()->odometry.resetPosition(getYaw(), getModulePositions(), pose));
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : mSwerveMods) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro() {
        gyro.setYaw(0);
    }

    public Rotation2d getYaw() {
        return (swerveConstants.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw())
                : Rotation2d.fromDegrees(gyro.getYaw());
    }

    public void resetModulesToAbsolute() {
        for (SwerveModule mod : mSwerveMods) {
            mod.resetToAbsolute();
        }
    }

    public double[] getAdvantageModuleStates() {
        double[] states = new double[8];
        for (int i = 0; i < 4; i++) {
            states[mSwerveMods[i].moduleNumber + i] = mSwerveMods[i].getState().angle.getDegrees();
            states[mSwerveMods[i].moduleNumber + i + 1] = mSwerveMods[i].getState().speedMetersPerSecond;
        }
        return states;
    }

    public double[] getAdvantageDesiredModuleStates() {
        double[] states = new double[8];
        for (int i = 0; i < 4; i++) {
            states[mSwerveMods[i].moduleNumber + i] = mSwerveMods[i].getDesierdState().angle.getDegrees();
            states[mSwerveMods[i].moduleNumber + i + 1] = mSwerveMods[i].getDesierdState().speedMetersPerSecond;
        }
        return states;
    }

    public ChassisSpeeds getRobotVelocity() {
        return swerveConstants.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    @Override
    public void periodic() {

        SmartDashboard.putNumber("vel", getRobotVelocity().vxMetersPerSecond);
        
        SmartDashboard.putNumber("error", headingPid.getPositionError());

        poseEstimation.update(getYaw(), getModulePositions());
        odometry.update(getYaw(), getModulePositions());

        vision.getRightEstimatedGlobalPose().ifPresent(
                est -> {
                    var estPose = est.estimatedPose.toPose2d();
                    // Change our trust in the measurement based on the tags we can see
                    var estStdDevs = vision.getEstimationStdDevsRight(estPose);

                    poseEstimation.addVisionMeasurement(
                            new Pose2d(estPose.getTranslation(), getYaw()), est.timestampSeconds, estStdDevs);
                });

        field2d.setRobotPose(poseEstimation.getEstimatedPosition());
        SmartDashboard.putNumber("dis", getPose().getTranslation().getDistance(FieldConstants.Speaker.centerSpeakerOpening.getTranslation()));

    }

    public void refreshClosedLoopRamp(double openLoopRampTime) {
        for (SwerveModule module : mSwerveMods) {
            module.refreshClosedLoopRamp(openLoopRampTime);
        }
    }

    public void stopModules() {
        for (SwerveModule module : mSwerveMods) {
            module.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)), true);
        }
    }

    public Command stopModulescCommand() {
        return runOnce(() -> {
            stopModules();
        });
    }

    /** Runs forwards at the commanded voltage. */
    public void runCharacterizationVolts(double volts) {
        for (SwerveModule mod : mSwerveMods) {
            mod.runCharacterizationVolts(volts);
        }
    }

    /** Returns the average drive velocity in radians/sec. */
    public double getCharacterizationVelocity() {
        double driveVelocityAverage = 0.0;
        for (var module : mSwerveMods) {
            driveVelocityAverage += module.getCharacterizationVelocity();
        }
        return driveVelocityAverage / 4.0;
    }

    public void setHeading(Rotation2d heading) {
        headingSupplier = heading;
    }

    public void disableHeading() {
        headingSupplier = null;
    }

    public Command setHeadingCommand(Rotation2d heading) {
        return Commands.runOnce(() -> headingSupplier = heading);
    }

    public Command disableHeadingCommand() {
        return Commands.runOnce(() -> headingSupplier = null);
    }
}