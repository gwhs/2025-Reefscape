package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedSubsystem extends SubsystemBase {
  AddressableLED m_led;
  AddressableLEDBuffer m_LedBuffer;

  public LedSubsystem() {
    m_led = new AddressableLED(LedUtils.PWM_LED_PORT);
    m_LedBuffer = new AddressableLEDBuffer(LedUtils.LED_LENGTH);

    m_led.setLength(m_LedBuffer.getLength());
    m_led.start();
  }

  public void setColor(LEDPattern pattern) {
    pattern.applyTo(m_LedBuffer);
    m_led.setData(m_LedBuffer);
  }

  public Color getColor(int index) {
    return m_LedBuffer.getLED(index);
  }

  public void turnOff() {
    setColor(LEDPattern.solid(Color.kBlack));
  }

  public Command setPattern(LEDPattern pattern) {
    return this.runOnce(() -> setColor(pattern));
  }
}
