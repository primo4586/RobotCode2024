package frc.robot.autos;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.Utils.AllianceFlipUtil;
import frc.robot.aRobotOperations.CollectToFeeder;
import frc.robot.aRobotOperations.InakeSafe;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.Supplier;

public class AutoCommands {

  public static Command moveWhileShooting() {
    return SwerveSubsystem.getInstance()
        .setHeadingCommand(AutoCommands::calculateShootHeading)
        .andThen(Commands.waitSeconds(0.7))
        .andThen(SwerveSubsystem.getInstance().disableHeadingCommand());
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

  public interface Region {
    boolean contains(Translation2d point);
  }

  public static class RectangularRegion implements Region {
    public final Translation2d topLeft;
    public final Translation2d bottomRight;

    public RectangularRegion(Translation2d topLeft, Translation2d bottomRight) {
      this.topLeft = topLeft;
      this.bottomRight = bottomRight;
    }

    public RectangularRegion(Translation2d center, double width, double height) {
      topLeft = new Translation2d(center.getX() - width / 2, center.getY() + height / 2);
      bottomRight = new Translation2d(center.getX() + width / 2, center.getY() - height / 2);
    }

    public boolean contains(Translation2d point) {
      return point.getX() >= topLeft.getX()
          && point.getX() <= bottomRight.getX()
          && point.getY() <= topLeft.getY()
          && point.getY() >= bottomRight.getY();
    }
  }

  public record CircularRegion(Translation2d center, double radius) implements Region {
    public boolean contains(Translation2d point) {
      return center.getDistance(point) <= radius;
    }
  }
}
