package frc.robot.subsystems;

import  edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class LEDSubsystem extends SubsystemBase {

  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;
  private int m_rainbowFirstPixelHue;


  // Store what the last hue of the first pixel is
  // private int m_rainbowFirstPixelHue;

  public LEDSubsystem(int id) {

    m_led = new AddressableLED(id);
    // Reuse buffer
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
    m_ledBuffer = new AddressableLEDBuffer(60);
    m_led.setLength(m_ledBuffer.getLength());

  }

  public void setColor(int red, int green, int blue)
  {
    Color color = new Color(red, green, blue);
    for (var i = 0; i < m_ledBuffer.getLength(); i++) 
    {
      m_ledBuffer.setLED(i, color);
    }
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  public Color getColor(int index)
  {
    return m_ledBuffer.getLED(index);
  }

  public void stopColor ()
  {
    Color color = new Color(0,0,0);
      for (var i = 0; i < m_ledBuffer.getLength(); i++) 
    {
      m_ledBuffer.setLED(i, color);
    }
    m_led.setData(m_ledBuffer);
    m_led.start();
  }
  
  public void rainbow() {
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
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  public void yellowWave() {
    Color yellow = new Color(255,0,255);
    Color off = new Color(0,0,0);
      for (var i = 1; i < m_ledBuffer.getLength(); i++) 
    {
      if(i < m_ledBuffer.getLength())
      {
        m_ledBuffer.setLED(i, yellow);
        m_ledBuffer.setLED(i - 1, off); 
      }
    m_led.setData(m_ledBuffer);
    m_led.start();
    }
  }
}
