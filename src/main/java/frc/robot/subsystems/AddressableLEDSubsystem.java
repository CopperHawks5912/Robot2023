// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.PWMConstants;

public class AddressableLEDSubsystem extends SubsystemBase {
  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;
  private int m_rainbowFirstPixelHue;
  private int m_ledMode;

  public AddressableLEDSubsystem() {
    m_led = new AddressableLED(PWMConstants.kLEDStringID);
    m_ledBuffer = new AddressableLEDBuffer(LEDConstants.kLEDStringLength);
    
    m_led.setLength(m_ledBuffer.getLength());

    // Set the data
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(m_ledMode == LEDConstants.kLEDModeRainbow)
      rainbowMode();
    else if(m_ledMode == LEDConstants.kLEDModeCone)
      coneMode();
    else if(m_ledMode == LEDConstants.kLEDModeCube)
      cubeMode();
    else if(m_ledMode == LEDConstants.kLEDModeAllianceBlue)
      blueMode();
    else if(m_ledMode == LEDConstants.kLEDModeAllianceRed)
      redMode();
    m_led.setData(m_ledBuffer);
   }

  public void setLEDMode(int mode)
  {
    m_ledMode = mode;  
  }
  private void blueMode() {
    // For every pixel
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setLED( i, Color.kBlue);
    }
  }
  private void redMode() {
    // For every pixel
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setLED( i, Color.kRed);
    }
  }
  private void coneMode() {
    // For every pixel
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB( i, 255, 68, 0);
    }
  }
  private void cubeMode() {
    // For every pixel
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, 100, 0, 100);
    }
  }
  private void rainbowMode() {
    // For every pixel
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
      // Set the value
      m_ledBuffer.setHSV(i, hue, 255, 128);
    }
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue += 3;
    // Check bounds
    m_rainbowFirstPixelHue %= 180;
  }


 

}
