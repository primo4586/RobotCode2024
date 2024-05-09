// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

/** Add your docs here. */
public class Shooter_Constants {

    // Shooter Motor IDs
    public static final int UP_MOTOR_SHOOTER_ID = 6;
    public static final int DOWN_MOTOR_SHOOTER_ID = 7;


    // Motion Magic Values
    public static final int MOTION_MAGIC_ACCELERATION = 300;
    public static final int MOTION_MAGIC_JERK = 3000;
    public static final double SENSOR_TO_MEC_RATIO = 0.5;
    public static final int MAX_VEL_ERROR = 2;

    
    // PID values for up motor
    public static final double UP_KP = 0.056;
    public static final double UP_KD = 0.0;
    public static final double UP_KS = 0.16;
    public static final double UP_KA = 0.0;

    // PID values for down motor
    public static final double DOWN_KP = 0.022;
    public static final double DOWN_KD = 0.0;
    public static final double DOWN_KS = 0.2998046875;
    public static final double DOWN_KA = 0.079175;
            
    //shooter velocity
    public static final double SPEAKR_VELOCITY = 60;
    public static final double AMP_VELOCITY = 23;
}
