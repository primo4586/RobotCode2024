// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooterArm;

import com.ctre.phoenix6.signals.ReverseLimitValue;

import frc.robot.util.exterpolation.ExterpolationMap;



/** Add your docs here. */
public class shooterArmConstants {

    // Technical Constants
    public static final int MOTOR_ID = 5;
    public static final double GEAR_RATIO = 100.0 / 22.0 * 100.0;
    public static final double SENSOR_TO_MEC_RATIO = GEAR_RATIO / 360.0;

    // Condition Constants
    public static final double MINIMUM_ERROR = 2;

    // MotionMagic Constants
    public static final double MM_CRUISE = 80;
    public static final double MM_ACCELERATION = 300;
    public static final double MM_JERK = 1600;

    public static final double KP = 0.1;
    public static final double KD = 0.0;
    public static final double KS = 0.032658;
    public static final double KA = 0.001121;
    public static final double KV = 0.13707;

    // Constant Limit Values
    public static final double FORWARD_LIMIT = 80;
    public static final double BACKWARD_LIMIT = -1;

    // ArmPoseReset Constant
    public static final double RESET_POSE = 0.0;
    public static final double RESET_SPEED = -0.1;
    
    public static final double SHOOT_BASE_ANGLE = 26;
    public static final double SHOOT_STAGE_ANGLE = 56;
    public static final double AMP_ANGLE = 32.4;
    
    public static final ReverseLimitValue lIMIT_SWITCH_TRUE_VALUE = ReverseLimitValue.ClosedToGround;

    
    public static final ExterpolationMap SPEAKER_ANGLE_EXTERPOLATION = 
    new ExterpolationMap().put(2.9, 62.0)
    .put(2.52, 55.0)
    .put(2.15, 50.0)
    .put(3.59, 63.0)
    .put(3.04, 56.0);
}

