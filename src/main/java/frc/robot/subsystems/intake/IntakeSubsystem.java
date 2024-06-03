// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Misc;

/**
 * The IntakeSubsystem is a class that controls the TalonFX motor for the
 * intake subsystem.
 */
public class IntakeSubsystem extends SubsystemBase implements IntakeConstants {
    private TalonFX m_motor = new TalonFX(MOTOR_ID, Misc.CAN_BUS_NAME);
    private DigitalInput m_opticSensor = new DigitalInput(OPTIC_SENSOR_ID);
    private TorqueCurrentFOC currentFOC = new TorqueCurrentFOC(0);

    private boolean hasNote = true;

    private static IntakeSubsystem INSTANCE;

    /**
     * Returns the instance of this subsystem.
     *
     * @return The instance of this subsystem
     */
    public static IntakeSubsystem getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new IntakeSubsystem();
        }
        return INSTANCE;
    }

    /**
     * Constructs a new IntakeSubsystem.
     */
    private IntakeSubsystem() {
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();

        // Set current limit parameters
        motorConfig.CurrentLimits.SupplyCurrentLimit = 40;
        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        motorConfig.CurrentLimits.SupplyCurrentThreshold = 100;
        motorConfig.CurrentLimits.SupplyTimeThreshold = 0.1;

        // Set motor output parameters
        motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        m_motor.getConfigurator().apply(motorConfig);
    }

    /**
     * Sets the motor to the specified current. 
     * IMPORTANT: This method is more accurate than setting the motor speed.
     *
     * @param current The current to set the motor to (in amps)
     * @return A command to set the motor to the specified current
     */
    public Command setCurrentCommand(double current) {
        // Create a command that sets the motor to the specified current
        // using the TalonFX's TorqueCurrentFOC control mode
        // The command will execute once and then finish
        return this.runOnce(() -> {
            m_motor.setControl(currentFOC.withOutput(current));
        });
    }

    /**
     * Sets the motor to the specified current.
     * IMPORTANT: This method is more accurate than setting the motor speed.
     *
     * @param current The current to set the motor to (in amps)
     */
    private void setCurrent(double current) {
        // Set the motor control to use the provided current
        // using the TalonFX's TorqueCurrentFOC control mode
        m_motor.setControl(currentFOC.withOutput(current));
    }


    /**
     * Command to intake the note.
     *
     * @return A command to intake the note
     */
    public Command intakeUntilNoteCommand() {
        return this.runEnd(() -> setCurrent(INTAKE_CURRENT), // run
                () -> {// end
                    m_motor.stopMotor();
                    hasNote = getOpticSensorValue();
                })
                .until(() -> getOpticSensorValue());
    }

    /**
     * Command to feed the shooter.
     *
     * @return A command to feed the shooter
     */
    public Command feedShooterCommand() {
        hasNote = false;
        return this.runEnd(() -> setCurrent(FEED_SHOOTER_CURRENT), () -> m_motor.stopMotor())
                .withTimeout(FEED_SHOOTER_TIME);
    }

    /**
     * Command to set the speed of the motor.
     *
     * @param speed The speed to set the motor to
     * @return A command to set the speed of the motor
     */
    public Command setSpeedCommand(double speed) {
        return this.runOnce(() -> m_motor.set(speed));
    }

    /**
     * Returns the value of the optic sensor.
     *
     * @return The value of the optic sensor
     */
    public boolean getOpticSensorValue() {
        return m_opticSensor.get();
    }

    /**
     * do not trust this to check if the note left the shooter only if a note has
     * entered
     *
     * @return Whether or not the intake has a note
     */
    public boolean getHasNote() {
        return hasNote;
    }

    @Override
    public void periodic() {
        SignalLogger.writeBoolean("intake has note", hasNote);
        SignalLogger.writeBoolean("intake optic sensor", m_opticSensor.get());
    }
}
