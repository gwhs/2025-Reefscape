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
    m_led = new AddressableLED(LedConstants.PWM_LED_PORT);
    m_LedBuffer = new AddressableLEDBuffer(LedConstants.LED_LENGTH);

    m_led.setLength(m_LedBuffer.getLength());
    m_led.start();
  }

  /**
   * NOTE: probably only use LEDPattern.solid() <br>
   * NOTE: java color class and FRC color class are not the same <br>
   * NOTE: FRC: Color.kGreen <br>
   * NOTE: JAVA: Color.green <br>
   * NOTE: ONLY USE FRC COLOR!!! <br>
   *
   * @param pattern
   */
  public void setColor(LEDPattern pattern) {
    pattern.applyTo(m_LedBuffer);
    m_led.setData(m_LedBuffer);
  }

  /**
   * NOTE: returns a FRC Color object (see above for why that matters) <br>
   * NOTE: the length is set to 60 in constants so if you want the color at index 61 you'll get an
   * error <br>
   * NOTE: there are no negatives it goes from 0 - 60 inclusively
   *
   * @param index the index to get the color at
   * @return the color at the specified index
   */
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
