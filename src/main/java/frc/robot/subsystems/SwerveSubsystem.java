package frc.robot.subsystems;

import frc.Utils.swerve.SwerveModule;
import frc.Utils.vision.Vision;
import frc.robot.Constants.Swerve.Mod0;
import frc.robot.Constants.Swerve.Mod1;
import frc.robot.Constants.Swerve.Mod2;
import frc.robot.Constants.Swerve.Mod3;

import static frc.robot.Constants.Swerve.*;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

public class SwerveSubsystem extends SubsystemBase {
    public SwerveDrivePoseEstimator poseEstimation;
    public SwerveModule[] mSwerveMods;
    public TalonSRX talonSRX;
    public PigeonIMU gyro;
    public Vision vision;
    Field2d field2d = new Field2d();
    Rotation2d simYaw = new Rotation2d();

    private static SwerveSubsystem instance;

    // singelton
    public static SwerveSubsystem getInstance() {
        if (instance == null) {
            instance = new SwerveSubsystem();
        }
        return instance;
    }

    private SwerveSubsystem() {
        talonSRX = new TalonSRX(pigeonID);
        gyro = new PigeonIMU(talonSRX);
        gyro.configFactoryDefault();
        zeroGyro();
        
        field2d.
        SmartDashboard.putData("field", field2d);

        vision = Vision.getInstance();

        mSwerveMods = new SwerveModule[] {
                new SwerveModule(0, Mod0.constants),
                new SwerveModule(1, Mod1.constants),
                new SwerveModule(2, Mod2.constants),
                new SwerveModule(3, Mod3.constants)
        };
        poseEstimation = new SwerveDrivePoseEstimator(swerveKinematics, getYaw(), getModulePositions(),new Pose2d(2,2,new Rotation2d(0)));
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop, boolean scaled) {
        SwerveModuleState[] swerveModuleStates = swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation,
                        getYaw())
                        : new ChassisSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, maxSpeed);

        if(scaled){
            for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
            }
        }
        else{
            for (SwerveModule mod : mSwerveMods) {
            mod.scaledSetDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
            }
        }

    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        setModuleStates(swerveKinematics.toSwerveModuleStates(chassisSpeeds));
    }

    public Pose2d getPose() {
        return poseEstimation.getEstimatedPosition();
    }

    public void resetOdometry(Pose2d pose) {
        poseEstimation.resetPosition(getYaw(), getModulePositions(), pose);
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
        return (invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw())
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
        return swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    @Override
    public void periodic() {
        //poseEstimation.update(getYaw(), getModulePositions());
/*
        vision.getRightEstimatedGlobalPose().ifPresent(
                est -> {
                    var estPose = est.estimatedPose.toPose2d();
                    // Change our trust in the measurement based on the tags we can see
                    var estStdDevs = vision.getEstimationStdDevsRight(estPose);

                    poseEstimation.addVisionMeasurement(
                            est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
                });

        vision.getLeftEstimatedGlobalPose().ifPresent(
                est -> {
                    var estPose = est.estimatedPose.toPose2d();
                    // Change our trust in the measurement based on the tags we can see
                    var estStdDevs = vision.getEstimationStdDevsLeft(estPose);

                    poseEstimation.addVisionMeasurement(
                            est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
                });
                */

        for (SwerveModule mod : mSwerveMods) {
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Desired angle", mod.getDesierdState().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
        }
        SmartDashboard.putNumberArray("ModuleStates", getAdvantageModuleStates());
        SmartDashboard.putNumberArray("DesiredModuleStates", getAdvantageDesiredModuleStates());

        field2d.setRobotPose(poseEstimation.getEstimatedPosition());
    }

    public void stopModules() {
        for (SwerveModule module : mSwerveMods) {
            module.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)), true);
        }
    }

    public Command stopModulescCommand(){
        return runOnce(()->{
            stopModules();
        });
    }

    /** Runs forwards at the commanded voltage. */
    public void runCharacterizationVolts(Measure<Voltage> volts) {
        for (SwerveModule mod : mSwerveMods){
            mod.runCharacterizationVolts(volts);
        }
    }

      // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));
    
      // Create a new SysId routine for characterizing the drive.
  private final SysIdRoutine m_sysIdRoutine =
      new SysIdRoutine(
          // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
          new SysIdRoutine.Config(),
          new SysIdRoutine.Mechanism(
              // Tell SysId how to plumb the driving voltage to the motors.
              (Measure<Voltage> volts) -> {
                runCharacterizationVolts(volts);
              },
              // Tell SysId how to record a frame of data for each motor on the mechanism being
              // characterized.
              log -> {
                // Record a frame for the left motors.  Since these share an encoder, we consider
                // the entire group to be one motor.
                log.motor("FL")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            mSwerveMods[0].get() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(mSwerveMods[0].getPosition().distanceMeters, Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(mSwerveMods[0].getCharacterizationVelocity(), MetersPerSecond));
                        
                log.motor("FR")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            mSwerveMods[1].get() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(mSwerveMods[1].getPosition().distanceMeters, Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(mSwerveMods[1].getCharacterizationVelocity(), MetersPerSecond));
                        
                log.motor("BL")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            mSwerveMods[2].get() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(mSwerveMods[2].getPosition().distanceMeters, Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(mSwerveMods[2].getCharacterizationVelocity(), MetersPerSecond));
                        
                log.motor("BR")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            mSwerveMods[3].get() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(mSwerveMods[3].getPosition().distanceMeters, Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(mSwerveMods[3].getCharacterizationVelocity(), MetersPerSecond));
                
              },
              // Tell SysId to make generated commands require this subsystem, suffix test state in
              // WPILog with this subsystem's name ("drive")
              this));
              
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
      return m_sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction);
    }

    public SwerveModuleState[] getSimStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getDesierdState();
        }
        return states;
    }

    SwerveModulePosition simModPose = new SwerveModulePosition();
    SwerveModulePosition[] poses = new SwerveModulePosition[]{
        simModPose,simModPose,simModPose,simModPose
    };
    Rotation2d rotation2d = new Rotation2d();
    double distance = 0;
    @Override
    public void simulationPeriodic() {
        super.simulationPeriodic();

        ChassisSpeeds chassisSpeed =  swerveKinematics.toChassisSpeeds(getSimStates());

        simYaw = Rotation2d.fromRadians(chassisSpeed.omegaRadiansPerSecond * 0.02);
        
        simModPose.angle = getSimStates()[0].angle;
        simModPose.distanceMeters += getSimStates()[0].speedMetersPerSecond * 0.02;
        
        for (SwerveModulePosition pose : poses) {
            pose = simModPose;
        }
        
        poseEstimation.update(simYaw, poses);

        field2d.setRobotPose(getPose());
    }
}