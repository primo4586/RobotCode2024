package frc.robot.autos;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.Utils.AllianceFlipUtil;
import frc.robot.Constants.FeederConstants;
import frc.robot.aRobotOperations.CollectToFeeder;
import frc.robot.aRobotOperations.InakeSafe;
import frc.robot.autos.Regions.Region;
import frc.robot.basicCommands.feederCommands.FeederSetSpeed;
import frc.robot.subsystems.SwerveSubsystem;


import java.util.function.Supplier;

public class AutoCommands {

  public static Command moveWhileShooting() {
    return SwerveSubsystem.getInstance()
        .setHeadingCommand(AutoCommands::calculateShootHeading)
          .alongWith(new AutoShooterSpeaker())
        .andThen(Commands.waitSeconds(0.7))
        .andThen(SwerveSubsystem.getInstance().disableHeadingCommand()
          .alongWith(new FeederSetSpeed(FeederConstants.FeederShootSpeed).deadlineWith(Commands.waitSeconds(FeederConstants.TimeToFeed))));
  }

  private static Rotation2d calculateShootHeading() {
    Twist2d fieldVel = SwerveSubsystem.getInstance().fieldVelocity();
    return ShotCalculator.calculate(
        AllianceFlipUtil.apply(FieldConstants.Speaker.centerSpeakerOpening.getTranslation()),
        SwerveSubsystem.getInstance().getPose().getTranslation(),
        new Translation2d(fieldVel.dx, fieldVel.dy))
        .goalHeading();
  }

  public static boolean inRegion(Supplier<Region> region) {
    return region.get().contains(SwerveSubsystem.getInstance().getPose().getTranslation());
  }

  public static Command waitForRegion(Supplier<Region> region) {
    return Commands.waitUntil(() -> inRegion(region));
  }

  public static Command intakeWhileInRegion(Supplier<Region> region) {
    return Commands.sequence(
        Commands.waitUntil(() -> inRegion(region)),
        new CollectToFeeder(),
        Commands.waitUntil(() -> !inRegion(region)),
        new InakeSafe());
  }

}
