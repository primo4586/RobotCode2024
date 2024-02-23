package frc.robot;

import com.pathplanner.lib.path.GoalEndState;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.Utils.PathPlanner.PathPlannerHelper;
import frc.robot.Constants.IntakeArmConstants;
import frc.robot.aRobotOperations.CollectToFeeder;
import frc.robot.aRobotOperations.CollectToIntake;
import frc.robot.aRobotOperations.IntakeToFeeder;
import frc.robot.aRobotOperations.PrepareForShoot;
import frc.robot.aRobotOperations.ShootSpeaker;
import frc.robot.aRobotOperations.ShootTouchingBase;
import frc.robot.basicCommands.ClimbingCommands.ClimbingSetSpeed;
import frc.robot.basicCommands.IntakeArmCommands.IntakeArmDown;
import frc.robot.basicCommands.IntakeArmCommands.IntakeArmSetSpeed;
import frc.robot.basicCommands.IntakeArmCommands.IntakeArmUP;
import frc.robot.basicCommands.IntakeArmCommands.ZeroIntakeArm;
import frc.robot.basicCommands.IntakeCommands.IntakeSetSpeed;
import frc.robot.basicCommands.ShooterArmCommands.MoveShooterArmTo;
import frc.robot.basicCommands.ShooterArmCommands.ShooterArmSpeakerAngle;
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
    private final CommandXboxController operator = new CommandXboxController(1);
    private final CommandXboxController test = new CommandXboxController(2);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;
    PathPlannerHelper pathPlannerHelper = PathPlannerHelper.getInstace();

    /* Subsystems */
    private final SwerveSubsystem swerve = SwerveSubsystem.getInstance();
    private final IntakeSubsystem intake = IntakeSubsystem.getInstance();
    private final IntakeArmSubsystem intakeArm = IntakeArmSubsystem.getInstance();


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        swerve.setDefaultCommand(
             new TeleopSwerve(
                 swerve, 
                 () -> -driver.getRawAxis(translationAxis), 
                 () -> -driver.getRawAxis(strafeAxis), 
                 () -> -driver.getRawAxis(rotationAxis), 
                 true,
                 () -> driver.rightTrigger().getAsBoolean()
             )
        );


        //IntakeArmSubsystem.getInstance().setDefaultCommand(new IntakeArmSetSpeed(()-> -test.getRightX()));


        
        // ShooterArmSubsystem.getInstance().setDefaultCommand(new InstantCommand(()->{
        //      ShooterArmSubsystem.getInstance().setSpeedArm(()-> test.getLeftX() / 10);
        //  }, ShooterArmSubsystem.getInstance()));
        // test.b().onTrue(new ZeroShooterArm());

        // ShooterSubsystem.getInstance().setDefaultCommand(new InstantCommand(()->{
        //     ShooterSubsystem.getInstance().manualSetShooterSpeed(()-> test.getRightY());
        // }, ShooterSubsystem.getInstance()));

        // IntakeSubsystem.getInstance().setDefaultCommand(new IntakeSetSpeed(()-> -test.getLeftY()));

        
        // test.a().onTrue(new CollectToIntake());

        // Configure the button bindings
        // test.x().onTrue(new MoveShooterArmTo(29));

        // driver.leftBumper().onTrue(new FeederSetSpeed(()->1));
        // test.rightBumper().onTrue(new FeederSetSpeed(()->0));
        // driver.x().whileTrue(new InstantCommand(()-> ShooterSubsystem.getInstance().setShooterSpeed(100,100), ShooterSubsystem.getInstance()));
        // test.start().whileTrue(new InstantCommand(()-> ShooterSubsystem.getInstance().setShooterSpeed(0, 0), ShooterSubsystem.getInstance()));
        
        // driver.leftTrigger().whileTrue(new AlignToSpeaker());
        //driver.x().onTrue(new ShooterArmSpeakerAngle());
        // driver.b().onTrue(new CollectToFeeder());
        // driver.a().onTrue(new ShootSpeaker());

        // // test.rightBumper().whileTrue(new InstantCommand(()-> ShooterSubsystem.getInstance().setShooterSpeed(40, 40), ShooterSubsystem.getInstance()));
        driver.b().onTrue(new IntakeArmUP());
        driver.x().onTrue(new IntakeArmDown());
        driver.a().onTrue(new ZeroIntakeArm());
        driver.start().onTrue(new ZeroShooterArm());
        // // test.leftBumper().onTrue(new ShootTouchingBase());


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
        driver.y().onTrue(new InstantCommand(() -> swerve.zeroGyro()));
        driver.x().onTrue(new ShootTouchingBase());
        driver.rightTrigger().onTrue(new ShootSpeaker());
        driver.leftTrigger().toggleOnTrue(new AlignToSpeaker());


        /* Operator Buttons */
        operator.x().onTrue(new CollectToFeeder());
        operator.y().toggleOnTrue(new PrepareForShoot());
        operator.a().onTrue(Commands.runOnce(()-> intakeArm.moveArmTo(IntakeArmConstants.SafeSetPoint), intakeArm));
        operator.rightBumper().onTrue(new IntakeArmDown());
        operator.leftBumper().onTrue(new IntakeArmUP());
        
        IntakeSubsystem.getInstance().setDefaultCommand(new IntakeSetSpeed(()-> -test.getLeftY()));
        FeederSubsystem.getInstance().setDefaultCommand(new FeederSetSpeed(()-> -test.getLeftY()));
    }
}
