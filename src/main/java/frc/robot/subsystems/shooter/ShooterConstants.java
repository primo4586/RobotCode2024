// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;


/** Add your docs here. */
public class ShooterConstants {

    public static class shooterConstants {
        // Shooter Motor IDs
        public static final int UP_MOTOR_SHOOTER_ID = 42;
        public static final int DOWN_MOTOR_SHOOTER_ID = 37;

        // Motion Magic Values
        public static final int MOTION_MAGIC_CRUISE_VELOCITY = 80;
        public static final int MOTION_MAGIC_ACCELERATION = 160;
        public static final int MOTION_MAGIC_JERK = 1600;

        public static final double PEAK_FORWARD_VOLTAGE = 11.5;
        public static final double PEAK_REVERSE_VOLTAGE = -11.5;

        public static final double GEAR_RATIO = 0.5;

        public static final int MAX_ERROR = 2;

        // PID values for up motor
        public static final double UP_KP = 0.056;
        public static final double UP_KD = 0.0;
        public static final double UP_KS = 0.16;
        public static final double UP_KV = 0.056;
        public static final double UP_KA = 0.0;

        // PID values for down motor
        public static final double DOWN_KP = 0.022;
        public static final double DOWN_KD = 0.0;
        public static final double DOWN_KS = 0.2998046875;
        public static final double DOWN_KV = 0.0583;
        public static final double DOWN_KA = 0.079175;

        public static final double SHOOT_BASE_SPEED = 50;
        public static final double SHOOT_STAGE_SPEED = 70;
        public static final double SHOOT_SPEED = 70;
        public static final double AMP_SPEED = 23;
    }
}
