package frc.robot.subsystems.endEffector;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

class EndEffectorIOSim implements EndEffectorIO {
  private FlywheelSim motor =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(1), 0.001, 1),
          DCMotor.getKrakenX60(1));

  @Override
  public Command setVoltage(double voltage) {
    return Commands.runOnce(() -> motor.setInputVoltage(voltage));
  }

  @Override
  public Command stopMotor() {
    return Commands.runOnce(() -> motor.setInputVoltage(0));
  }

  @Override
  public void update() {
    motor.update(.020);
  }
}
