// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.util.PathPlanner;

import java.util.List;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;

import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.swerve.SwerveConstants.AutoConstants;
import frc.util.AllianceFlipUtil;

/** Add your docs here. */
public class PathPlannerHelper {

    private final SwerveSubsystem swerve;

    private static PathPlannerHelper instance = new PathPlannerHelper();

    public static PathPlannerHelper getInstace() {
        if (instance == null) {
            instance = new PathPlannerHelper();
        }
        return instance;
    }

    private PathPlannerHelper() {
        swerve = SwerveSubsystem.getInstance();

        AutoBuilder.configureHolonomic(
                swerve::getPose,
                swerve::resetPose,
                swerve::getRobotVelocity,
                swerve::setChassisSpeeds,
                new HolonomicPathFollowerConfig(
                        AutoConstants.XY_PID,
                        AutoConstants.rotation_PID,
                        AutoConstants.kMaxSpeedMetersPerSecond,
                        AutoConstants.driveBaseRadius,
                        AutoConstants.replanningConfig),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                swerve);
    }

    public Command followPath(PathPlannerPath path) {
        if (path == null)
            return Commands.none();

        // Create a path following command using AutoBuilder. This will also trigger
        // event markers.
        return AutoBuilder.followPath(path);
    }

    public Command followPath(String pathName, boolean resetOdometry) {
        // Load the path you want to follow using its name in the GUI
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

            // Create a path following command using AutoBuilder. This will also trigger
            // event markers.
            if (resetOdometry) {

                return swerve.resetOdometry(AllianceFlipUtil.apply(path.getPreviewStartingHolonomicPose()))
                        .andThen(AutoBuilder.followPath(path));
            }
            return AutoBuilder.followPath(path);

        } catch (Exception e) {
            SmartDashboard.putString("faild" + pathName, pathName);
            return Commands.none();
        }
    }

    public Command followChoreoPath(String ChoreoTrajectory, boolean resetOdometry) {
        // Load the path you want to follow using its name in the GUI
        try {
            PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory(ChoreoTrajectory);

            // Create a path following command using AutoBuilder. This will also trigger
            // event markers.
            if (resetOdometry) {
                return swerve.resetOdometry(AllianceFlipUtil.apply(path.getPreviewStartingHolonomicPose()))
                        .andThen(AutoBuilder.followPath(path));
            }
            return AutoBuilder.followPath(path);
        } catch (Exception e) {
            SmartDashboard.putString("faild" + ChoreoTrajectory, ChoreoTrajectory);
            return Commands.none();
        }
    }

    public PathPlannerPath generatePath(List<Translation2d> bezierPoints, GoalEndState goalEndState) {
        return new PathPlannerPath(
                bezierPoints,
                AutoConstants.pathConstraints,
                goalEndState);
    }

    public Command generateAndFollowPath(List<Translation2d> bezierPoints, GoalEndState goalEndState) {
        return Commands.runOnce(() -> {
            followPath(generatePath(bezierPoints, goalEndState));
        });
    }

    public Command generateAndFollowPath(Translation2d endPoint, GoalEndState goalEndState) {
        return Commands.runOnce(() -> {
            Translation2d currentPose = swerve.getPose().getTranslation();
            Rotation2d rotation = angleBetweenPoints(currentPose, endPoint);
            // rotation = Rotation2d.fromDegrees(0);
            Pose2d startPose = new Pose2d(currentPose, rotation);
            Pose2d endPos = new Pose2d(endPoint, rotation);

            followPath(generatePath(PathPlannerPath.bezierFromPoses(startPose, endPos), goalEndState)).schedule();
        });
    }

    public Rotation2d angleBetweenPoints(Translation2d point1, Translation2d point2) {
        Translation2d anglePoint = point1.minus(point2);

        // Calculate the angle in radians
        double angleRad = Math.atan2(anglePoint.getY(), anglePoint.getX());

        // Ensure the angle is between 0 and 2π (0 and 360 degrees)
        if (angleRad < 0) {
            angleRad += 2 * Math.PI;
        }

        return new Rotation2d(angleRad);
    }
}
