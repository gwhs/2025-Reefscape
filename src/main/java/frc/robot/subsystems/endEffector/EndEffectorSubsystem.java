package frc.robot.subsystems.endEffector;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

  public Command setVoltage(double voltage) {
    return Commands.runOnce(() -> endEffectorIO.setVoltage(voltage));
  }

  public Command stopMotor() {
    return Commands.runOnce(() -> endEffectorIO.stopMotor());
  }

  @Override
  public void periodic() {
    endEffectorIO.update();
  }
}
