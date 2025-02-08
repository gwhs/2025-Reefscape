package frc.robot.subsystems.endEffector;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

class EndEffectorIOSim implements EndEffectorIO {
  private FlywheelSim motor =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(1), 0.001, 1),
          DCMotor.getKrakenX60(1));

  @Override
  public void setVoltage(double voltage) {
    motor.setInputVoltage(voltage);
  }

  @Override
  public void stopMotor() {
    motor.setInputVoltage(0);
  }

  @Override
  public double getVelocity() {
    return motor.getAngularVelocity().in(RotationsPerSecond);
  }

  @Override
  public double getVoltage() {
    return motor.getInputVoltage();
  }

  @Override
  public void update() {
    motor.update(.020);
  }
}
