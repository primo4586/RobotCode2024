package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.SwerveConstants.swerveConstants;
import frc.utils.motors.CANSparkMaxUtil;
import frc.utils.motors.FalconConversions;
import frc.utils.motors.CANSparkMaxUtil.Usage;
import frc.utils.swerve.CTREModuleState;
import frc.utils.swerve.SwerveModuleConstants;

public class SwerveModule {
    public int moduleNumber;
    private Rotation2d angleOffset;
    private Rotation2d lastAngle;

    private CANSparkMax mAngleMotor;
    private RelativeEncoder integratedEncoder;
    private TalonFX mDriveMotor;
    private CANcoder mCancoder;

    private SwerveModuleState desiredState = new SwerveModuleState(0, Rotation2d.fromDegrees(0));

    /* drive motor control requests */
    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
    private final MotionMagicVelocityVoltage driveVelocity = new MotionMagicVelocityVoltage(0, 0, false, 0,
            0, true, false, false);

    /* angle motor control requests */
    private SparkPIDController anglePosition;

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;

        /* Angle Encoder Config */
        mCancoder = new CANcoder(moduleConstants.cancoderID, Constants.CAN_BUS_NAME);
        configAngleEncoder();

        /* Angle Motor Config */
        mAngleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        anglePosition = mAngleMotor.getPIDController();
        integratedEncoder = mAngleMotor.getEncoder();
        configAngleMotor();

        /* Drive Motor Config */
        mDriveMotor = new TalonFX(moduleConstants.driveMotorID, Constants.CAN_BUS_NAME);
        configDriveMotor();

        lastAngle = getState().angle;
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        /*
         * This is a custom optimize function, since default WPILib optimize assumes
         * continuous controller which CTRE and Rev onboard is not
         */
        desiredState = CTREModuleState.optimize(desiredState, getState().angle);
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
        this.desiredState = desiredState;
    }

    public void scaledSetDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        /*
         * This is a custom optimize function, since default WPILib optimize assumes
         * continuous controller which CTRE and Rev onboard is not
         */
        desiredState = CTREModuleState.optimize(desiredState, getState().angle);
        desiredState.speedMetersPerSecond *= desiredState.angle.minus(getAngle()).getCos();
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
        this.desiredState = desiredState;
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            driveDutyCycle.Output = desiredState.speedMetersPerSecond / swerveConstants.maxSpeed;
            mDriveMotor.setControl(driveDutyCycle);
        } else {
            driveVelocity.Velocity = FalconConversions.MPSToTalon(desiredState.speedMetersPerSecond,
                    swerveConstants.wheelCircumference, swerveConstants.driveGearRatio);
            mDriveMotor.setControl(driveVelocity);
        }
        this.desiredState = desiredState;
    }

    private void setAngle(SwerveModuleState desiredState) {
        // Prevent rotating module if speed is less then 1%. Prevents jittering.
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (swerveConstants.maxSpeed * 0.01))
                ? lastAngle
                : desiredState.angle;

        anglePosition.setReference(angle.getDegrees(), ControlType.kPosition);
        lastAngle = angle;
        this.desiredState = desiredState;
    }

    private Rotation2d getAngle() {
        return Rotation2d.fromDegrees(mAngleMotor.getEncoder().getPosition());
    }

    public Rotation2d getCANcoder() {
        return Rotation2d.fromRotations(mCancoder.getAbsolutePosition().getValue());
    }

    public void resetToAbsolute() {
        // mCancoder.getPosition().setUpdateFrequency(50);
        double absolutePosition = mCancoder.getAbsolutePosition().waitForUpdate(250).getValue() * 360
                - angleOffset.getDegrees();
        System.out.println(integratedEncoder.setPosition(absolutePosition));
        // mCancoder.getPosition().setUpdateFrequency(0);
        // mCancoder.optimizeBusUtilization();
    }

    private void configAngleEncoder() {
        mCancoder.getPosition().setUpdateFrequency(50);
        mCancoder.getConfigurator().apply(Robot.ctreConfigs.swerveCANcoderConfig);
    }

    private void configAngleMotor() {
        mAngleMotor.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(mAngleMotor, Usage.kPositionOnly);
        mAngleMotor.setSmartCurrentLimit(swerveConstants.angleCurrentLimit);
        mAngleMotor.setInverted(swerveConstants.angleMotorInvert);
        mAngleMotor.setIdleMode(swerveConstants.angleNeutralMode);
        integratedEncoder.setPositionConversionFactor(swerveConstants.angleGearRatio);
        anglePosition.setP(swerveConstants.angleKP);
        anglePosition.setI(swerveConstants.angleKI);
        anglePosition.setD(swerveConstants.angleKD);
        anglePosition.setFF(swerveConstants.angleKFF);
        anglePosition.setOutputRange(-1, 1);
        mAngleMotor.enableVoltageCompensation(swerveConstants.voltageComp);
        System.out.println("burn " + moduleNumber + mAngleMotor.burnFlash());
        Timer.delay(1);
        resetToAbsolute();
    }

    private void configDriveMotor() {
        mDriveMotor.getConfigurator().apply(Robot.ctreConfigs.swerveDriveFXConfig);
        mDriveMotor.getConfigurator().setPosition(0);
    }

    public void refreshClosedLoopRamp(double openLoopRampTime) {
        OpenLoopRampsConfigs rampsConfigs = new OpenLoopRampsConfigs();
        rampsConfigs.DutyCycleOpenLoopRampPeriod = openLoopRampTime;
        rampsConfigs.VoltageOpenLoopRampPeriod = openLoopRampTime;
        mDriveMotor.getConfigurator().refresh(rampsConfigs);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                FalconConversions.talonToMPS(mDriveMotor.getVelocity().getValue(), swerveConstants.wheelCircumference,
                        swerveConstants.driveGearRatio),
                getAngle());
    }

    public SwerveModuleState getDesierdState() {
        return desiredState;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                FalconConversions.talonToMeters(mDriveMotor.getPosition().getValue(),
                        swerveConstants.wheelCircumference, swerveConstants.driveGearRatio),
                getAngle());
    }

    public void runCharacterizationVolts(Double voltage) {
        anglePosition.setReference(0, ControlType.kPosition);
        mDriveMotor.setVoltage(voltage);
    }

    public double getCharacterizationVelocity() {
        return FalconConversions.talonToMPS(mDriveMotor.getVelocity().getValue(),
                swerveConstants.wheelCircumference,
                swerveConstants.driveGearRatio);
    }

    public double get() {
        return mDriveMotor.get();
    }
}