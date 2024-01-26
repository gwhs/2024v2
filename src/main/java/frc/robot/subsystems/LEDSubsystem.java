package frc.robot.subsystems;

import  edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class LEDSubsystem extends SubsystemBase {

  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;
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

  public void turnOffLED() //not working
  {
    m_led.stop();
  }
}
