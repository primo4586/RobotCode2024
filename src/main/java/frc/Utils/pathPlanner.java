// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.Utils;

import java.util.List;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;

import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.SwerveSubsystem;

/** Add your docs here. */
public class pathPlanner {

    private final SwerveSubsystem swerve;

    private static pathPlanner instance;

    public static pathPlanner getInstace() {
        if (instance == null) {
            instance = new pathPlanner();
        }
        return instance;
    }

    private pathPlanner() {
        swerve = SwerveSubsystem.getInstance();

        AutoBuilder.configureHolonomic(
                swerve::getPose,
                swerve::resetOdometry,
                swerve::getRobotVelocity,
                swerve::setChassisSpeeds,
                new HolonomicPathFollowerConfig(
                        AutoConstants.XY_PID,
                        AutoConstants.rotation_PID,
                        Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                        AutoConstants.driveBaseRadius,
                        AutoConstants.replanningConfig),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
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
    
    public void followPath(PathPlannerPath path) {

        // Create a path following command using AutoBuilder. This will also trigger
        // event markers.
        AutoBuilder.followPath(path).schedule();
    }

    public void followPath(String pathName) {
        // Load the path you want to follow using its name in the GUI
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

        // Create a path following command using AutoBuilder. This will also trigger
        // event markers.
        AutoBuilder.followPath(path).schedule();
    }
    
    public void followChoreoPath(String ChoreoTrajectory) {
        // Load the path you want to follow using its name in the GUI
        PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory(ChoreoTrajectory);

        // Create a path following command using AutoBuilder. This will also trigger
        // event markers.
        AutoBuilder.followPath(path).schedule();
    }

    public void pathFind(Pose2d targPose, PathConstraints constraints, double endVel, double rotationDelayDistance) {
        AutoBuilder.pathfindToPose(targPose, constraints, endVel, rotationDelayDistance).schedule();
    }
    
    public void pathFind(Pose2d targetPose) {
        pathFind(targetPose, AutoConstants.pathConstraints, 0, 0);
    }

    public PathPlannerPath generatePath(List<Translation2d> bezierPoints, GoalEndState goalEndState) {// TODO: constants
        return new PathPlannerPath(
            bezierPoints,
            AutoConstants.pathConstraints, 
                goalEndState);
    }

    public PathPlannerPath generatePath(List<Translation2d> bezierPoints, Rotation2d goalEndRotation) {
        return generatePath(bezierPoints, new GoalEndState(0, goalEndRotation));
    }

    public void generateAndFollowPath(List<Translation2d> bezierPoints, GoalEndState goalEndState) {
        followPath(generatePath(bezierPoints, goalEndState));
    }
}
