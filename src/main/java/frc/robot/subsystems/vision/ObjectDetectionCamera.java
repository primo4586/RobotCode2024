// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;

/** Add your docs here. */
public class ObjectDetectionCamera {
    private final PhotonCamera camera;

    public ObjectDetectionCamera(String cameraName) {
        camera = new PhotonCamera(cameraName);
    }

    /**
     * Get if the camera is detecting an object
     * @return true if the camera is detecting an object
     */
    public boolean getDetectingObject() {
        return camera.getLatestResult().hasTargets();
    }

    /**
     * Get the angle from the target
     * @return the angle from the target or 0 if no target
     */
    public double getAngleFromTarget() {
        return getDetectingObject() ? camera.getLatestResult().getBestTarget().getYaw() : 0;
    }
}
