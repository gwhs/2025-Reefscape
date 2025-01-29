package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;
import edu.wpi.first.wpilibj.util.Color;

public class LedIOSim implements LedIO {
  AddressableLEDSim sim;
  AddressableLEDBuffer buf;

  public LedIOSim() {
    sim = new AddressableLEDSim();
    buf = new AddressableLEDBuffer(LedConstants.LED_LENGTH);
  }

  public void setColor(LEDPattern pattern) {
    pattern.applyTo(buf);
  }

  public Color getColor(int index) {
    return Color.kAquamarine;
  }

  public void turnOff() {}

  public void update() {}
}
