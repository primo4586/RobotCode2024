// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.swerve.TunerConstants;
import frc.robot.subsystems.vision.ObjectDetectionCamera;
import frc.robot.subsystems.vision.Vision_Constants;

/** Add your docs here. */
public interface Misc {

    String CAN_BUS_NAME = "CANBUS";

    Translation2d speakerPoseBlue = new Translation2d(0, 5.54);
    Translation2d speakerPoseRed = new Translation2d(16.39, 5.54);

    DoubleSupplier distanceFromSpeaker = () -> TunerConstants.Swerve.getPose().getTranslation().getDistance(
            DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? speakerPoseBlue
                    : speakerPoseRed);

    ObjectDetectionCamera objectDetectionCamera = new ObjectDetectionCamera(Vision_Constants.K_NOTE_CAMERA_NAME);

    BooleanSupplier isWithinShootingRange = () -> distanceFromSpeaker.getAsDouble() <= 5;
}
