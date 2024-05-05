// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooterArm;

import com.ctre.phoenix6.signals.ReverseLimitValue;

/** Add your docs here. */
public interface shooterArmConstants {

    // Technical Constants
    int MOTOR_ID = 5;
    double GEAR_RATIO = 100.0 / 22.0 * 100.0;
    double SENSOR_TO_MEC_RATIO = GEAR_RATIO / 360.0;

    // Condition Constants
    double MINIMUM_ERROR = 2;

    // MotionMagic Constants
    double MM_CRUISE = 80;
    double MM_ACCELERATION = 300;
    double MM_JERK = 1600;

    double KP = 0.1;
    double KD = 0.0;
    double KS = 0.032658;
    double KA = 0.001121;
    double KV = 0.13707;

    // Constant Limit Values
    double FORWARD_LIMIT = 80;
    double BACKWARD_LIMIT = -1;

    // ArmPoseReset Constant
    double RESET_POSE = 0.0;
    double RESET_SPEED = -0.1;
    
    double SHOOT_BASE_ANGLE = 26;
    double SHOOT_STAGE_ANGLE = 56;
    double AMP_ANGLE = 32.4;
    
    ReverseLimitValue lIMIT_SWITCH_TRUE_VALUE = ReverseLimitValue.ClosedToGround;
}
