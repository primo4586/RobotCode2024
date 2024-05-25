package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public interface Vision_Constants {

    String K_RIGHT_CAMERA_NAME = "YOUR CAMERA NAME";
    String K_LEFT_CAMERA_NAME = "YOUR CAMERA NAME";
    String K_NOTE_CAMERA_NAME = "YOUR CAMERA NAME";

    Transform3d K_RIGHT_ROBOT_TO_CAM = new Transform3d(new Translation3d(0.5, 0.0, 0.5),
            new Rotation3d(0, 0, 0));

    Transform3d K_LEFT_ROBOT_TO_CAM = new Transform3d(new Translation3d(0.5, 0.0, 0.5),
            new Rotation3d(0, 0, 0));


    // The layout of the AprilTags on the field
    AprilTagFieldLayout K_TAG_LAYOUT = AprilTagFields.kDefaultField.loadAprilTagLayoutField();
    // The standard deviations of our vision estimated poses, which affect
    // correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    Matrix<N3, N1> K_SINGLE_TAG_STD_DEVS = VecBuilder.fill(4, 4, 8);
    Matrix<N3, N1> K_MULTI_TAG_STD_DEVS = VecBuilder.fill(0.5, 0.5, 1);

}