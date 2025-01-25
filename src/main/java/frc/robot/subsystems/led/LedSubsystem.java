package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedSubsystem extends SubsystemBase {
  AddressableLED m_led;
  public AddressableLEDBuffer m_LedBuffer;

  public LedSubsystem() {
    m_led = new AddressableLED(9);
    m_LedBuffer = new AddressableLEDBuffer(60);

    m_led.setLength(m_LedBuffer.getLength());
    m_led.start();
  }

  public void setColor(LEDPattern pattern, AddressableLEDBuffer buf) {
    pattern.applyTo(m_LedBuffer);
    m_led.setData(buf);
  }

  public Color getColor(AddressableLEDBuffer buf, int index) {

    return buf.getLED(index);
  }

  public void turnOff() {
    setColor(LEDPattern.solid(Color.kBlack), m_LedBuffer);
  }

 
}
