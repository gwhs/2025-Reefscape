package frc.robot.subsystems.endEffector;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

class EndEffectorIOTalon implements EndEffectorIO {

  private TalonFX motor = new TalonFX(EndEffectorConstants.deviceID, "rio");
  private final StatusSignal<Voltage> volts = motor.getMotorVoltage();
  private final StatusSignal<AngularVelocity> velocity = motor.getVelocity();
  private final StatusSignal<Temperature> temperature = motor.getDeviceTemp();

  @Override
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  @Override
  public void stopMotor() {
    motor.setVoltage(0);
  }

  @Override
  public double getVelocity() {
    return velocity.getValueAsDouble();
  }

  @Override
  public double getVoltage() {

    return volts.getValueAsDouble();
  }

  @Override
  public void update() {
    boolean endEffectorConnected =
        (BaseStatusSignal.refreshAll(volts, velocity, temperature)).isOK();
    DogLog.log("EndEffector/Temperature", temperature.getValueAsDouble());
    DogLog.log("endEffector/Connected", endEffectorConnected);
  }
}
