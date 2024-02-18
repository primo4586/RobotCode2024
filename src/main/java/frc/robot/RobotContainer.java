package frc.robot;

import com.pathplanner.lib.path.GoalEndState;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.Utils.PathPlanner.PathPlannerHelper;
import frc.robot.aRobotOperations.CollectToFeeder;
import frc.robot.aRobotOperations.CollectToIntake;
import frc.robot.aRobotOperations.IntakeToFeeder;
import frc.robot.basicCommands.IntakeArmCommands.IntakeArmSetSpeed;
import frc.robot.basicCommands.IntakeArmCommands.ZeroIntakeArm;
import frc.robot.basicCommands.IntakeCommands.IntakeSetSpeed;
import frc.robot.basicCommands.ShooterArmCommands.MoveShooterArmTo;
import frc.robot.basicCommands.ShooterArmCommands.ZeroShooterArm;
import frc.robot.basicCommands.ShooterCommands.ShooterSetSpeed;
import frc.robot.basicCommands.SwerveCommands.*;
import frc.robot.basicCommands.feederCommands.FeedToShooter;
import frc.robot.basicCommands.feederCommands.FeederSetSpeed;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final CommandXboxController driver = new CommandXboxController(0);
    
    private final CommandXboxController test = new CommandXboxController(2);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;
    PathPlannerHelper pathPlannerHelper = PathPlannerHelper.getInstace();

    /* Subsystems */
    private final SwerveSubsystem swerve = SwerveSubsystem.getInstance();


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        swerve.setDefaultCommand(
             new TeleopSwerve(
                 swerve, 
                 () -> -driver.getRawAxis(translationAxis), 
                 () -> -driver.getRawAxis(strafeAxis), 
                 () -> -driver.getRawAxis(rotationAxis), 
                 true,
                 () -> driver.rightBumper().getAsBoolean()
             )
        );

        test.a().toggleOnTrue(null );

        // IntakeArmSubsystem.getInstance().setDefaultCommand(new IntakeArmSetSpeed(()-> -test.getRightX()));
        // // test.a().onTrue(new ZeroIntakeArm());

        
        // ShooterArmSubsystem.getInstance().setDefaultCommand(new InstantCommand(()->{
        //     ShooterArmSubsystem.getInstance().setSpeedArm(()-> test.getLeftX());
        // }, ShooterArmSubsystem.getInstance()));
        // test.b().onTrue(new ZeroShooterArm());

        // ShooterSubsystem.getInstance().setDefaultCommand(new InstantCommand(()->{
        //     ShooterSubsystem.getInstance().manualSetShooterSpeed(()-> test.getRightY());
        // }, ShooterSubsystem.getInstance()));

        // FeederSubsystem.getInstance().setDefaultCommand(new FeederSetSpeed(()-> -test.getLeftY()));
        // IntakeSubsystem.getInstance().setDefaultCommand(new IntakeSetSpeed(()-> -test.getLeftY()));

        // test.b().onTrue(new CollectToFeeder());
        
        // test.b().onTrue(new CollectToIntake());

        // Configure the button bindings
        // test.a().onTrue(new ZeroShooterArm());
        // test.x().onTrue(new MoveShooterArmTo(10));

        // test.a().whileTrue(new InstantCommand(()-> ShooterSubsystem.getInstance().setShooterSpeed(120, 120), ShooterSubsystem.getInstance()));
        // test.x().whileTrue(new InstantCommand(()-> ShooterSubsystem.getInstance().setShooterSpeed(0, 0), ShooterSubsystem.getInstance()));
        test.a().onTrue(new ZeroIntakeArm());


        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * 
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        //TODO: add back
        // driver.y().onTrue(new InstantCommand(() -> swerve.zeroGyro()));

        // driver.a().onTrue(pathPlannerHelper.generateAndFollowPath(new Translation2d(2, 2),
        //         new GoalEndState(0, new Rotation2d(0))));


        // driver.b().onTrue(pathPlannerHelper.generateAndFollowPath(PathPlannerPath.bezierFromPoses(
        //     new Pose2d(swerve.getPose().getTranslation(),new Rotation2d()),
        //     new Pose2d(swerve.getPose().getTranslation().plus(new Translation2d(2.0, 0.0)), new Rotation2d())),
        //         new GoalEndState(0, new Rotation2d(0))));

    }
}
