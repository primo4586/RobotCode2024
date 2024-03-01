package frc.robot.basicCommands.SwerveCommands;

import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.swerve.SwerveConstants.swerveConstants;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;


public class TeleopSwerve extends Command {    
    private SwerveSubsystem swerve = SwerveSubsystem.getInstance();    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private Boolean fieldSentric;
    private BooleanSupplier slowMode;

    public TeleopSwerve(DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, boolean fieldSentric, BooleanSupplier slowMode) {
        addRequirements(swerve);

        this.slowMode = slowMode;
        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.fieldSentric = fieldSentric;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);

        /* Drive */
        if (!slowMode.getAsBoolean()){
            swerve.drive(
                new Translation2d(translationVal, strafeVal).times(swerveConstants.maxSpeed), 
                rotationVal * swerveConstants.maxAngularVelocity, 
                fieldSentric, 
                true
            );
        /* Slow mode drive */
        }else{
            swerve.drive(
                new Translation2d(translationVal, strafeVal).times(swerveConstants.maxSpeed * swerveConstants.XYSlowRatio), 
                rotationVal * swerveConstants.maxAngularVelocity * swerveConstants.rotationSlowRatio, 
                fieldSentric, 
                true
            );
        }
    }
}