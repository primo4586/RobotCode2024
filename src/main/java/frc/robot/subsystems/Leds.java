// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.LedsConstants;

public class Leds extends SubsystemBase {
  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;

  private static Leds instance;

  public static Leds getInstance() {
    if (instance == null) {
      instance = new Leds();
    }
    return instance;
  }

  /** Creates a new Leds. */
  public Leds() {
    AddressableLED m_led = new AddressableLED(LedsConstants.LedsPort);
    AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(LedsConstants.LedsLength);

    m_led.setLength(m_ledBuffer.getLength());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void turnOffLed() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++)
      m_ledBuffer.setRGB(i,0,0,0);
    
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  public void turnLedsByRGB(int red, int green, int blue) {
    for (var i = 0; i < m_ledBuffer.getLength(); i++)
      m_ledBuffer.setRGB(i,red,green,blue);
    
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  public void turnLedsByRGBRange(int red, int green, int blue, int minLed, int maxLed) {
    for (var i = minLed; i <= maxLed; i++)
      m_ledBuffer.setRGB(i,red,green,blue);
    
    m_led.setData(m_ledBuffer);
    m_led.start();
  }



}
