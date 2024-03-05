package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.basicCommands.LedsCommands.BasicLedsCommand;
import frc.robot.subsystems.Leds.Leds;
import frc.robot.subsystems.swerve.SwerveSubsystem;
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

    Leds leds = Leds.getInstance();

    /* Subsystems */
    private final SwerveSubsystem swerve = SwerveSubsystem.getInstance();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        //swerve.setDefaultCommand(
        //        new TeleopSwerve(
        //                () -> -driver.getHID().getLeftY(),
        //                () -> -driver.getHID().getLeftX(),
        //                () -> -driver.getHID().getRightX(),
        //                true,
        //               () -> driver.getHID().getRightTriggerAxis() > 0.5));

        leds.setDefaultCommand(new BasicLedsCommand());

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
        // driverStart.onTrue(new EStop());
        // driverYTrigger.onTrue(new InstantCommand(() -> swerve.zeroGyro()));
        // driverXTrigger.onTrue(new ShootTouchingBase());
        // driverRightBumperTrigger.whileTrue(new FeederSetSpeed(()->1).repeatedly());
        // driverLeftTriggerToggle.toggleOnTrue(new AlignToSpeaker());
        // driverBackTrigger.onTrue(swerve.disableHeadingCommand());
        // driverPovLeftTrigger.onTrue(new ZeroIntakeArm());
        // driverPovRightTrigger.onTrue(new ZeroShooterArm());

        // // Operator Button Bindings
        // operatorStart.onTrue(new EStop());
        // operatorXTrigger.onTrue(new CollectToFeeder());
        // operatorYTrigger.toggleOnTrue(new PrepareForShoot().repeatedly());
        // operatorATrigger
        //         .onTrue(Commands.runOnce(() -> intakeArm.moveArmTo(IntakeArmConstants.SafeSetPoint), intakeArm));
        // operatorRightBumperTrigger.onTrue(new IntakeArmDown());
        // operatorLeftBumperTrigger.onTrue(new IntakeArmUP());
        // operatorBTrigger.onTrue(
        //         new InstantCommand(() -> ShooterSubsystem.getInstance().coast(), ShooterSubsystem.getInstance()));

        // IntakeSubsystem.getInstance().setDefaultCommand(new IntakeSetSpeed(() -> -operator.getHID().getLeftY()));
        // FeederSubsystem.getInstance().setDefaultCommand(new FeederSetSpeed(() -> -operator.getHID().getLeftY()));
    }
}
