package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

public interface LedIO {

  void setColor(LEDPattern pattern);

  Color getColor(int index);

  void turnOff();

  void update();
}
