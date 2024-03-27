package frc.robot.basicCommands.SwerveCommands;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.swerve.SwerveConstants.swerveConstants;
import frc.util.vision.Vision;

/*
 * Turn to note is one of our best codes so far. It is written by one of the 
 * best programmer we have! mister Eilon Nave Huppert.
 * this code is programmed to detectd notes with the help of our Vision class and turn to position
 * where our great XXXfeederXXX intake face them.
 */

public class TurnToNote extends Command {
  SwerveSubsystem swerve = SwerveSubsystem.getInstance();
  Vision vision = Vision.getInstance();
  public PIDController notepid = swerveConstants.notePid;


  public TurnToNote() {
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    notepid.setTolerance(3);
  }

  @Override
  public void execute() {
    double angelFromNote = vision.GetAngelFromNote();
    swerve.drive(new Translation2d(),notepid.calculate(angelFromNote,0),true,false);
  }

  @Override
  public void end(boolean interrupted) {
    swerve.drive(new Translation2d(),0,true,false);
  }

  @Override
  public boolean isFinished() {
     return notepid.atSetpoint();
  }
}
