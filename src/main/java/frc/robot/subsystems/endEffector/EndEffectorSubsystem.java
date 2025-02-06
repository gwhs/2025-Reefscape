package frc.robot.subsystems.endEffector;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EndEffectorSubsystem extends SubsystemBase {

  EndEffectorIO endEffectorIO;

  public EndEffectorSubsystem() {

    if (RobotBase.isSimulation()) {
      endEffectorIO = new EndEffectorIOSim();
    } else {
      endEffectorIO = new EndEffectorIOSparkMax();
      // endEffectorIO = new EndEffectorIOTalon();
    }
  }

  public void setVoltage(double voltage) {
    endEffectorIO.setVoltage(voltage);
  }

  public void stopMotor() {
    endEffectorIO.stopMotor();
  }

  @Override
  public void periodic() {
    endEffectorIO.update();
  }
}
