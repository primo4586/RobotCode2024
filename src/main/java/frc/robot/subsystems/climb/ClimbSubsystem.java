// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;

/**
 * The ClimbSubsystem is a class that controls the two motors on the
 * climb subsystem.
 */
public class ClimbSubsystem extends SubsystemBase implements ClimbConstants{
    private final CANSparkMax m_RightSparkMax = new CANSparkMax(M_CLIMB_RIGHT_MOTOR_ID, MotorType.kBrushless);
    private final CANSparkMax m_LeftSparkMax = new CANSparkMax(M_CLIMB_LEFT_MOTOR_ID, MotorType.kBrushless);

    private static ClimbSubsystem INSTANCE;

    /**
     * Gets the ClimbSubsystem instance. If it hasn't been created yet,
     * create it.
     * 
     * @return the ClimbSubsystem instance
     */
    public static ClimbSubsystem getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new ClimbSubsystem();
        }
        return INSTANCE;
    }

    /**
     * Creates a new ClimbSubsystem.
     */
    private ClimbSubsystem() {
        m_RightSparkMax.setSmartCurrentLimit(CURRENT_LIMIT);
        m_LeftSparkMax.setSmartCurrentLimit(CURRENT_LIMIT);

        m_RightSparkMax.setIdleMode(IdleMode.kBrake);
        m_LeftSparkMax.setIdleMode(IdleMode.kBrake);
    }

    /**
     * Creates a command that makes the climb motors move.
     * 
     * @param speed the speed to move the motors at
     * @return the command
     */
    public Command moveClimb(DoubleSupplier speed) {
        return run(() -> {
            m_RightSparkMax.set(speed.getAsDouble());
            m_LeftSparkMax.set(speed.getAsDouble());
        });
    }

    /**
     * Creates a command that makes the climb motors move with custom left and right
     * speeds.
     *
     * @param leftSpeed  a DoubleSupplier that provides the speed for the left motor
     * @param rightSpeed a DoubleSupplier that provides the speed for the right
     *                   motor
     * @return a Command that moves the climb motors with the provided speeds
     */
    public Command moveClimb(DoubleSupplier leftSpeed, DoubleSupplier rightSpeed) {
        return run(() -> {
            m_RightSparkMax.set(leftSpeed.getAsDouble());
            m_LeftSparkMax.set(rightSpeed.getAsDouble());
        });
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
