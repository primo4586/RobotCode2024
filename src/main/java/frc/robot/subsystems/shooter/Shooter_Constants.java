// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

/** Add your docs here. */
public interface Shooter_Constants {

    // Shooter Motor IDs
    int UP_MOTOR_SHOOTER_ID = 6;
    int DOWN_MOTOR_SHOOTER_ID = 7;


    // Motion Magic Values
    int MOTION_MAGIC_ACCELERATION = 300;
    int MOTION_MAGIC_JERK = 3000;
    double SENSOR_TO_MEC_RATIO = 0.5;
    int MAX_VEL_ERROR = 2;

    
    // PID values for up motor
    double UP_KP = 0.056;
    double UP_KD = 0.0;
    double UP_KS = 0.16;
    double UP_KV = 0.056;
    double UP_KA = 0.0;

    // PID values for down motor
    double DOWN_KP = 0.022;
    double DOWN_KD = 0.0;
    double DOWN_KS = 0.2998046875;
    double DOWN_KV = 0.0583;
    double DOWN_KA = 0.079175;
            
    //shooter velocity
    double SPEAKR_VELOCITY = 60;
    double AMP_VELOCITY = 23;
}
