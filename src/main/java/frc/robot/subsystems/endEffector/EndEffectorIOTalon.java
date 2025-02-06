package frc.robot.subsystems.endEffector;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

class EndEffectorIOTalon implements EndEffectorIO {

  private TalonFX motor = new TalonFX(EndEffectorConstants.deviceID, "rio");
  private final StatusSignal<Voltage> volts = motor.getMotorVoltage();
  private final StatusSignal<Temperature> temp = motor.getDeviceTemp();

  @Override
  public Command setVoltage(double voltage) {
    return Commands.runOnce(() -> motor.setVoltage(voltage));
  }

  @Override
  public Command stopMotor() {
    return Commands.runOnce( () -> motor.setVoltage(0));
  }

  @Override
  public void update() {
    boolean endEffectorConnected = (BaseStatusSignal.refreshAll(volts, temp)).isOK();

    DogLog.log("EndEffector/Temp", temp.getValueAsDouble());
    DogLog.log("EndEffector/Voltage", volts.getValueAsDouble());
    DogLog.log("EndEffector/Connected", endEffectorConnected);
  }
}
