// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.takeFeed.TakeFeedSubsystem;

public class Leds extends SubsystemBase {
  private TakeFeedSubsystem takeFeed = TakeFeedSubsystem.getInstance();
  private int counter = 0;

  private AddressableLED m_led;
  private static AddressableLEDBuffer m_ledBuffer;

  private static Leds instance;

  public static Leds getInstance() {
    if (instance == null) {
      instance = new Leds();
    }
    return instance;
  }

  /** Creates a new Leds. */
  public Leds() {
    m_led = new AddressableLED(LedsConstants.LedsPort);
    m_ledBuffer = new AddressableLEDBuffer(LedsConstants.LedsLength);

    m_led.setLength(m_ledBuffer.getLength());

    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void turnOffLed() {
    for (var i = 1; i < LedsConstants.LedsLength; i++)
      m_ledBuffer.setRGB(i, 0, 0, 0);

    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  public void turnLedsByRGB(int red, int green, int blue) {
    for (var i = 1; i < LedsConstants.LedsLength; i++)
      m_ledBuffer.setRGB(i, red, green, blue);

    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  public void turnLedsByRGBRange(int red, int green, int blue, int minLed, int maxLed) {
    for (int i = minLed; i <= maxLed; i++)
      m_ledBuffer.setRGB(i, red, green, blue);

    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  public void ledsByNotePose() {
    if (takeFeed.getOpticSensorValue()) {
      turnLedsByRGBRange(0, 0, 0, 0, 0);
    }
  }

  public void LedsRainbow(int delay) {
    while (true) {
      for (int i = 0; i < LedsConstants.LedsLength; i++) {
      final int hue = (counter + (i * 180 / LedsConstants.LedsLength)) % 180;
      m_ledBuffer.setHSV(i, hue, 255, 128);
    }

    counter += 3;
    counter %= 180;

    m_led.setData(m_ledBuffer);
    m_led.start();

    Timer.delay(delay / 1000.0);
    }
    
  }

  // Wave effect method with adjustable speed
  public void waveEffect(int delay) {
    while (true) {
      // Turn the whole row blue one by one until the end
      for (int i = 0; i < LedsConstants.LedsLength; i++) {
        // Set the LED color to blue
        m_ledBuffer.setRGB(i, 0, 191, 230);

        // Set the LEDs
        m_led.setData(m_ledBuffer);
        m_led.start();

        // Delay to control the speed of the wave effect
        Timer.delay(delay / 1000.0); // Convert delay to seconds
      }

      // Turn each LED orange one by one
      for (int i = 0; i < LedsConstants.LedsLength; i++) {
        // Set the LED color to orange
        m_ledBuffer.setRGB(i, 255, 65, 0);

        // Set the LEDs
        m_led.setData(m_ledBuffer);
        m_led.start();

        // Delay to control the speed of the wave effect
        Timer.delay(delay / 1000.0); // Convert delay to seconds
      }
    }
  }

  public void fireEffect(int delay) {

    // orange rgb - (255,69, 0);
    // our orange rgb - 0xFF3300

    while (true) {
      // Clear the LED buffer to black
      // Set the flame effect
      for (int i = 0; i < LedsConstants.LedsLength; i++) {
          int intensity = (int) (Math.sin((i / 5.0) + Timer.getFPGATimestamp() * 2.0) * 128.0 + 128.0);
          // Adjust intensity to control the brightness
          intensity = Math.min(255, Math.max(0, intensity));


          // Set the LED color directly using RGB
          m_ledBuffer.setRGB(i, 0, (intensity * 191) / 255, (intensity * 255) / 255);
      }

      // Set the LEDs
      m_led.setData(m_ledBuffer);
      m_led.start();

      // Delay to control the speed of the flame effect
      Timer.delay(delay / 1000.0); // Convert delay to seconds
  }
  }




  public void LedSunset(int delay) {

    // orange rgb - (255,69, 0);
    // our orange rgb - 0xFF3300

    while (true) {
      // Clear the LED buffer to black
      // Set the flame effect
      for (int i = 0; i < LedsConstants.LedsLength; i++) {
          int intensity = (int) (Math.sin((i / 10.0) + Timer.getFPGATimestamp() * 2.0) * 128.0 + 128.0);
          int blueIntensity = (int) (Math.sin((i / 10.0) + Timer.getFPGATimestamp() * 2.0 + Math.PI) * 128.0 + 128.0);

          // Adjust intensity to control the brightness
          intensity = Math.min(255, Math.max(0, intensity));
          blueIntensity = Math.min(255, Math.max(0, blueIntensity));


          // Set the LED color directly using RGB
          m_ledBuffer.setRGB(i, (intensity * 255) / 255, (intensity * 69) / 255, blueIntensity);
      }

      // Set the LEDs
      m_led.setData(m_ledBuffer);
      m_led.start();

      // Delay to control the speed of the flame effect
      Timer.delay(delay / 1000.0); // Convert delay to seconds
  }
  }
}
