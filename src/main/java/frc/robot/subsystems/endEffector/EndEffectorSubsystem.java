package frc.robot.subsystems.endEffector;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EndEffectorSubsystem extends SubsystemBase {

  EndEffectorIO eeio;

  public EndEffectorSubsystem() {

    if (RobotBase.isSimulation()) {
      eeio = new EndEffectorIOSim();
    } else {
      eeio = new EndEffectorIOSparkMax();
   // eeio = new EndEffectorIOTalon();
    }
  }

  public void setVoltage(double voltage) {
    eeio.setVoltage(voltage);
  }

  public void stopMotor() {
    eeio.stopMotor();
  }

  @Override
  public void periodic() {
    eeio.update();
  }
}
