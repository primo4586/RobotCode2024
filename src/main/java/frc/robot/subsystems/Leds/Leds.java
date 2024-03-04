// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.takeFeed.TakeFeedSubsystem;

public class Leds extends SubsystemBase {
  private TakeFeedSubsystem takeFeed = TakeFeedSubsystem.getInstance();
  private Timer timer;
  private int counter = 0;

  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;

  private static Leds instance;

  private final ShooterSubsystem shooter = ShooterSubsystem.getInstance();

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

  public void ledsByNotePose(){
    if (takeFeed.getOpticSensorValue()){
      turnLedsByRGBRange(0, 0, 0, 0, 0);
    }
  }

  public void LedWhileShooting(){
    for (int i = 0; i < LedsConstants.LedsLength; i++){
      turnLedsByRGBRange(40, 20, 150, i, i);
      timer.delay(0.3);   
    }
    for (int i = 0; i < LedsConstants.LedsLength; i++){
      turnLedsByRGBRange(0, 0, 150, i, i);
      timer.delay(0.3);   
    }
  }

  public void LedsRainbow(){
    
  }
}
