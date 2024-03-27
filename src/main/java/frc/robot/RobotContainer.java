package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.a_robotCommandGroups.ReadyShootSpeaker;
import frc.robot.a_robotCommandGroups.ShootBase;
import frc.robot.a_robotCommandGroups.ShootSpeaker;
import frc.robot.a_robotCommandGroups.ShootStage;
import frc.robot.basicCommands.ShooterArmCommands.ZeroShooterArm;
import frc.robot.basicCommands.ShooterCommands.ShooterSetSpeedForever;
import frc.robot.basicCommands.SwerveCommands.AutoAlignToSpeaker;
import frc.robot.basicCommands.SwerveCommands.TeleopSwerve;
import frc.robot.basicCommands.SwerveCommands.TurnToNote;
import frc.robot.basicCommands.TakeFeedCommands.CollectUntilNote;
import frc.robot.basicCommands.TakeFeedCommands.TakeFeedJoystickSetSpeed;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.takeFeed.TakeFeedSubsystem;
import frc.util.PathPlanner.PathPlannerHelper;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    // Driver Triggers
    Trigger driverYTrigger = driver.y();
    Trigger driverXTrigger = driver.x();
    Trigger driverBTrigger = driver.b();
    Trigger driverRightBumperTrigger = driver.rightBumper();
    Trigger driverLeftTriggerToggle = driver.leftTrigger();
    Trigger driverBackTrigger = driver.back();
    Trigger driverPovLeftTrigger = driver.povLeft();
    Trigger driverPovRightTrigger = driver.povRight();
    Trigger driverStart = driver.start();

    // Operator Triggers
    Trigger operatorXTrigger = operator.x();
    Trigger operatorYTrigger = operator.y();
    Trigger operatorATrigger = operator.a();
    Trigger operatorRightBumperTrigger = operator.rightBumper();
    Trigger operatorLeftBumperTrigger = operator.leftBumper();
    Trigger operatorBTrigger = operator.b();
    Trigger operatorStart = operator.start();

    PathPlannerHelper pathPlannerHelper = PathPlannerHelper.getInstace();

    /* Subsystems */
    private final SwerveSubsystem swerve = SwerveSubsystem.getInstance();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        swerve.setDefaultCommand(
                new TeleopSwerve(
                        () -> -driver.getHID().getLeftY(),
                        () -> -driver.getHID().getLeftX(),
                        () -> -driver.getHID().getRightX(),
                        true,
                        () -> driver.getHID().getRightTriggerAxis() > 0.5));


        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * 
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {

        // Driver Button Bindings
        // pathPlanner.followChoreoPath("2 to 1", false)
        //driverStart.onTrue(new EStop());
        //driverYTrigger.onTrue(new TurnToNote());
        driverYTrigger.onTrue(new InstantCommand(() -> swerve.zeroGyro()));
        driverXTrigger.onTrue(new ShootBase());
        driverBTrigger.onTrue(new ShootStage());
        driverRightBumperTrigger.whileTrue(new ShootSpeaker());
        driverLeftTriggerToggle.toggleOnTrue(new AutoAlignToSpeaker());
        driverBackTrigger.onTrue(swerve.disableHeadingCommand());
        driverPovRightTrigger.onTrue(new ZeroShooterArm());

        // // Operator Button Bindings
        // operatorRightBumperTrigger.whileTrue(new ShooterSetSpeedForever(ShooterConstants.shooterConstants.SHOOT_SPEED));
        // operatorLeftBumperTrigger.onTrue(new InstantCommand(()->ShooterSubsystem.getInstance().coast()));
        //operatorStart.onTrue(new EStop());
        operatorXTrigger.onTrue(new CollectUntilNote());
        operatorYTrigger.onTrue(new ReadyShootSpeaker());
        operatorBTrigger.onTrue(new InstantCommand(() -> ShooterSubsystem.getInstance().coast(), ShooterSubsystem.getInstance()));
        TakeFeedSubsystem.getInstance().setDefaultCommand(new TakeFeedJoystickSetSpeed(()-> - operator.getHID().getLeftY()));
    }
}
