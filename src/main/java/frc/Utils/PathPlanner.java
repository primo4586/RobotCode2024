// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.Utils;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.SwerveSubsystem;

/** Add your docs here. */
public class PathPlanner {

    private static PathPlanner instance;

    public static PathPlanner getInstace() {
        if (instance == null) {
            instance = new PathPlanner();
        }
        return instance;
    }

    SwerveSubsystem swerve;

    private PathPlanner() {
        swerve = SwerveSubsystem.getInstance();

        AutoBuilder.configureHolonomic(//TODO: constants
                swerve::getPose,
                swerve::resetOdometry,
                swerve::getRobotVelocity,
                swerve::setChassisSpeeds,
                new HolonomicPathFollowerConfig(
                        new PIDConstants(0, 0),
                        new PIDConstants(0, 0),
                        3.5,
                        Math.sqrt((0.615 / 2) * (0.615 / 2) + (0.615 / 2) * (0.615 / 2)),
                        new ReplanningConfig(true, true, 0.5,
                                1)),
                swerve);
    }

    public void followPath(PathPlannerPath path) {

        // Create a path following command using AutoBuilder. This will also trigger
        // event markers.
        AutoBuilder.followPathWithEvents(path).schedule();;
    }

    public void followPath(String pathName) {
        // Load the path you want to follow using its name in the GUI
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

        // Create a path following command using AutoBuilder. This will also trigger
        // event markers.
        AutoBuilder.followPathWithEvents(path).schedule();;
    }

    public void pathFind(Pose2d targPose, PathConstraints constraints, double endVel, double rotationDelayDistance) {
        AutoBuilder.pathfindToPose(targPose, constraints, endVel, rotationDelayDistance).schedule();
    }
    
    public void pathFind(Pose2d targetPose) {// TODO: constants
        pathFind(targetPose, new PathConstraints(4, 400, 4, 400), 0, 0);
    }

    public PathPlannerPath generatePath(List<Translation2d> bezierPoints, GoalEndState goalEndState) {// TODO: constants
        return new PathPlannerPath(
            bezierPoints, 
            null, 
                goalEndState);
    }

    public PathPlannerPath generatePath(List<Translation2d> bezierPoints, Rotation2d goalEndRotation) {
        return generatePath(bezierPoints, goalEndRotation);
    }

    public void generateAndFollowPath(List<Translation2d> bezierPoints, GoalEndState goalEndState) {
        followPath(generatePath(bezierPoints, goalEndState));
    }
}
