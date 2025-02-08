package frc.robot.subsystems.endEffector;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

class EndEffectorIOTalon implements EndEffectorIO {

  private TalonFX motor = new TalonFX(EndEffectorConstants.deviceID, "rio");
  private final StatusSignal<Voltage> volts = motor.getMotorVoltage();
  private final StatusSignal<AngularVelocity> velocity = motor.getVelocity();

  @Override
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  @Override
  public void stopMotor() {
    motor.setVoltage(0);
  }

  @Override
  @SuppressWarnings("unused") // motorOK is neverused
  public double getVelocity() {
      boolean motorOK = (BaseStatusSignal.refreshAll(velocity)).isOK();
      return velocity.getValueAsDouble();
  }

  @Override
  @SuppressWarnings("unused") // voltsOK is never used
  public double getVoltage() {
    boolean voltsOK = (BaseStatusSignal.refreshAll(volts)).isOK();
      return volts.getValueAsDouble();
  }

  @Override
  public void update() {
    // logged in subsystem
  }
}
