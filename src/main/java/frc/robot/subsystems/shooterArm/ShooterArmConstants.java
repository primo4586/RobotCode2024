// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooterArm;

import frc.util.interpolation.InterpolationMap;

public class ShooterArmConstants {

    public static class shooterArmConstants {

        // Technical Constants
        public static final int SHOOTER_ARM_ID = 5;
        public static final int SWITCH_ID = 9;
        public static final int ENCODER_COUNTS_PER_REVOLUTION = 1;
        public static final double GEAR_RATIO = 100.0 / 22.0 * 100.0;
        public static final double TICKS_PER_DEGREE = ENCODER_COUNTS_PER_REVOLUTION * GEAR_RATIO / 360.0;

        // Condition Constants
        public static final double MINIMUM_ERROR = 1;

        // MotionMagic Constants
        public static final double MM_CRUISE = 80;
        public static final double MM_ACCELERATION = 300;
        public static final double MM_JERK = 1600;

        public static final double KP = 0.1;
        public static final double KD = 0.0;
        public static final double KS = 0.032658;
        public static final double KA = 0.001121;
        public static final double KV = 0.13707;

        // MaxVol Constant
        public static final double PEEK_REVERSE_VOLTAGE = -11.5;
        public static final double PEEK_FORWARD_VOLTAGE = 11.5;

        // Constant Limit Values
        public static final double FORWARD_LIMIT = 80;
        public static final double BACKWARD_LIMIT = -1;

        // ArmPoseReset Constant
        public static final double RESET_POSE = 0.0;
        public static final double RESET_SPEED = -0.1;

        public static final double SHOOTER_ARM_START_POSE = 0;

        public static InterpolationMap SHOOTER_ANGLE_INTERPOLATION_MAP = new InterpolationMap()
        .put(2.9, 62)
        .put(2.52 , 55)
        .put(2.15 , 50)
        .put(3.59, 63)
        .put(3.04, 56);

        public static final double SHOOT_BASE_ANGLE = 26; //6.1064453125
        public static final double SHOOT_STAGE_ANGLE = 56;
        public static final double AMP_ANGLE = 0;
    }
}
