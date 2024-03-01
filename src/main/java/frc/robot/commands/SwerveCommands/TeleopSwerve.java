package frc.robot.commands.SwerveCommands;

import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.swerve.SwerveConstants.swerveConstants;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;


public class TeleopSwerve extends Command {    
    private SwerveSubsystem s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private Boolean fieldSentric;
    private BooleanSupplier slowMode;

    public TeleopSwerve(SwerveSubsystem s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, boolean fieldSentric, BooleanSupplier slowMode) {
        this.s_Swerve = s_Swerve;
        this.slowMode = slowMode;
        addRequirements(s_Swerve);

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
            s_Swerve.drive(
                new Translation2d(translationVal, strafeVal).times(swerveConstants.maxSpeed), 
                rotationVal * swerveConstants.maxAngularVelocity, 
                fieldSentric, 
                true,
                false
            );
        /* Slow mode drive */
        }else{
            s_Swerve.drive(
                new Translation2d(translationVal, strafeVal).times(swerveConstants.maxSpeed * swerveConstants.XYSlowRatio), 
                rotationVal * swerveConstants.maxAngularVelocity * swerveConstants.rotationSlowRatio, 
                fieldSentric, 
                true,
                false
            );
        }
    }
}