// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooterArm;

import com.ctre.phoenix6.signals.ReverseLimitValue;

import frc.robot.util.exterpolation.ExterpolationMap;


/**
 * Constants for the Shooter Arm Subsystem.
 */
public class shooterArmConstants {

    // Technical Constants
    /** The ID of the TalonFX motor controlling the shooter arm. */
    public static final int MOTOR_ID = 5;
    /** The gear ratio of the shooter arm. */
    public static final double GEAR_RATIO = 100.0 / 22.0 * 100.0;
    /** The ratio of sensor units to mechanical units. */
    public static final double SENSOR_TO_MEC_RATIO = GEAR_RATIO / 360.0;

    // Condition Constants
    /** The minimum position error to consider the shooter arm at the correct position. */
    public static final double MINIMUM_ERROR = 2;

    // MotionMagic Constants
    /** The cruise velocity of the shooter arm. */
    public static final double MM_CRUISE = 80;
    /** The acceleration of the shooter arm. */
    public static final double MM_ACCELERATION = 300;
    /** The jerk of the shooter arm. */
    public static final double MM_JERK = 1600;

    /** The PIDF gains of the shooter arm. */
    public static final double KP = 0.1;
    public static final double KD = 0.0;
    public static final double KS = 0.032658;
    public static final double KA = 0.001121;
    public static final double KV = 0.13707;

    // Constant Limit Values
    /** The forward limit of the shooter arm. */
    public static final double FORWARD_LIMIT = 80;
    /** The backward limit of the shooter arm. */
    public static final double BACKWARD_LIMIT = -1;

    // ArmPoseReset Constant
    /** The position to reset the shooter arm to. */
    public static final double RESET_POSE = 0.0;
    /** The speed to move the shooter arm when resetting. */
    public static final double RESET_SPEED = -0.1;

    /** The base angle of the shooter arm when shooting. */
    public static final double SHOOT_BASE_ANGLE = 26;
    /** The stage angle of the shooter arm when shooting. */
    public static final double SHOOT_STAGE_ANGLE = 56;
    /** The angle of the amp when shooting. */
    public static final double AMP_ANGLE = 32.4;

    /** The value of the limit switch when it is closed. */
    public static final ReverseLimitValue lIMIT_SWITCH_TRUE_VALUE = ReverseLimitValue.ClosedToGround;

    /** A map of speaker angles to the corresponding shooter arm angle. */
    public static final ExterpolationMap SPEAKER_ANGLE_EXTERPOLATION = 
    new ExterpolationMap().put(2.9, 62.0)
    .put(2.52, 55.0)
    .put(2.15, 50.0)
    .put(3.59, 63.0)
    .put(3.04, 56.0);
}

