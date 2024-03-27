package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public final class Constants {

    public static final double stickDeadband = 0.1;
    
    public static final String CAN_BUS_NAME = "canBus";



    public static class Vision {
        public static final String kRightCameraName = "right_Camera";
        public static final String kLeftCameraName = "left_Camera";
        public static final String kTestCameraName = "noteCamera";
        // Cam mounted facing forward, half a meter forward of center, half a meter up
        // from center.
        public static final Transform3d kRightRobotToCam = new Transform3d(new Translation3d(-0.16, -0.075, 0.45),
                new Rotation3d(- Units.degreesToRadians(-27.55), 0, Units.degreesToRadians(180)));

        public static final Transform3d kLeftRobotToCam = new Transform3d(new Translation3d(-0.171, 0.316, 0.40),
                new Rotation3d(Units.degreesToRadians(-20.2), 0, Units.degreesToRadians(180)));

        // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout kTagLayout = AprilTagFields.kDefaultField.loadAprilTagLayoutField();

        // The standard deviations of our vision estimated poses, which affect
        // correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static final Matrix<N3, N1> kRightSingleTagStdDevs = VecBuilder.fill(1, 1, 2);
        public static final Matrix<N3, N1> kRightMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

        public static final Matrix<N3, N1> kLeftSingleTagStdDevs = VecBuilder.fill(1, 1, 2);
        public static final Matrix<N3, N1> kLeftMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

        public static final Pose2d target = new Pose2d(1, 1, new Rotation2d(Units.degreesToRadians(0)));
    }

}
