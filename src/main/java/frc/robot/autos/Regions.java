// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */
public class Regions {

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
