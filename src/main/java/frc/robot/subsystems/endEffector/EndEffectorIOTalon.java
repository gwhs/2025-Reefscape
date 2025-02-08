package frc.robot.subsystems.endEffector;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

class EndEffectorIOTalon implements EndEffectorIO {

  private TalonFX motor = new TalonFX(EndEffectorConstants.deviceID, "rio");
  private final StatusSignal<Voltage> volts = motor.getMotorVoltage();
  private final StatusSignal<Temperature> temp = motor.getDeviceTemp();

  public EndEffectorIOTalon() {
    TalonFXConfiguration talonConfig = new TalonFXConfiguration();
    CurrentLimitsConfigs limitsConfigs = talonConfig.CurrentLimits;

    limitsConfigs.withStatorCurrentLimitEnable(true);
    limitsConfigs.withStatorCurrentLimit(15);

    StatusCode status = StatusCode.StatusCodeNotInitialized;

    for (int i = 0; i < 5; i++) {
      status = motor.getConfigurator().apply(talonConfig);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not configure device. Error: " + status.toString());
    }
  }

  @Override
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  @Override
  public void stopMotor() {
    motor.setVoltage(0);
  }

  @Override
  public void update() {
    boolean endEffectorConnected = (BaseStatusSignal.refreshAll(volts, temp)).isOK();

    DogLog.log("EndEffector/Temp", temp.getValueAsDouble());
    DogLog.log("EndEffector/Voltage", volts.getValueAsDouble());
    DogLog.log("EndEffector/Connected", endEffectorConnected);
  }
}
