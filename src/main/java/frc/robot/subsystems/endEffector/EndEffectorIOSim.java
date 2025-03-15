package frc.robot.subsystems.endEffector;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;

class EndEffectorIOSim implements EndEffectorIO {
  private FlywheelSim motor =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              DCMotor.getFalcon500Foc(1), 0.001, EndEffectorConstants.GEAR_RATIO),
          DCMotor.getFalcon500Foc(1));

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
  public void setAmps(double current, double dutyCycle) {
    double resistance = DCMotor.getFalcon500(1).rOhms;
    double voltage = current * resistance;
    motor.setInputVoltage(voltage);
  }

  private boolean coralSensorState = false;

  public EndEffectorIOSim() {
    SmartDashboard.putData(
        "Simulation/Coral Sensor", Commands.runOnce(() -> coralSensorState = !coralSensorState));
  }

  public boolean coralLoaded() {
    return coralSensorState;
  }

  @Override
  public void update() {
    motor.update(.020);
  }
}
