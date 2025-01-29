package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedSubsystem extends SubsystemBase {
  LedIO ledIO;

  public LedSubsystem() {
    if (RobotBase.isSimulation()) {
      ledIO = new LedIOSim();
    } else {
      ledIO = new LedIOReal();
    }
  }

  public void setColor(LEDPattern pattern) {
    ledIO.setColor(pattern);
  }

  public Color getColor(int index) {
    return ledIO.getColor(index);
  }

  public void turnOff() {
    ledIO.turnOff();
  }

  @Override
  public void periodic() {
    ledIO.update();
  }
}
