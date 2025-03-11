package frc.robot.subsystems.endEffector;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

class EndEffectorIOTalon implements EndEffectorIO {

  public TOFSensor m_coral_detector = new TOFSensor(EndEffectorConstants.CORAL_DETECTOR_ID);

  private TalonFX motor = new TalonFX(EndEffectorConstants.deviceID, "rio");
  private final StatusSignal<Voltage> volts = motor.getMotorVoltage();
  private final StatusSignal<AngularVelocity> velocity = motor.getVelocity();
  private final StatusSignal<Temperature> temperature = motor.getDeviceTemp();
  private final StatusSignal<Current> statorCurrent = motor.getStatorCurrent();
  private TorqueCurrentFOC currentControl = new TorqueCurrentFOC(0);

  private final Alert endEffectorMotorConnectedAlert =
      new Alert("End Effector Motor Not Connected", AlertType.kError);

  public EndEffectorIOTalon() {
    TalonFXConfiguration talonConfig = new TalonFXConfiguration();
    CurrentLimitsConfigs limitsConfigs = talonConfig.CurrentLimits;

    talonConfig.TorqueCurrent.withPeakForwardTorqueCurrent(40);
    talonConfig.TorqueCurrent.withPeakReverseTorqueCurrent(-40);

    limitsConfigs.withStatorCurrentLimitEnable(true);
    limitsConfigs.withStatorCurrentLimit(60);

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
  public double getVelocity() {
    return velocity.getValueAsDouble();
  }

  @Override
  public double getVoltage() {

    return volts.getValueAsDouble();
  }

  @Override
  public void setAmps(double current) {
    motor.setControl(currentControl.withOutput(current).withMaxAbsDutyCycle(.2));
  }

  public boolean coralLoaded() {
    // double distance = m_colorSensor.getProximity();
    double distance = m_coral_detector.getRange();
    if (distance < 30) {
      return true;
    } else {
      return false;
    }
  }

  @Override
  public void update() {
    boolean endEffectorConnected =
        (BaseStatusSignal.refreshAll(volts, velocity, temperature, statorCurrent)).isOK();
    DogLog.log("EndEffector/Temperature", temperature.getValueAsDouble());
    DogLog.log("EndEffector/Connected", endEffectorConnected);
    DogLog.log("EndEffector/StatorCurrent", statorCurrent.getValueAsDouble());
    endEffectorMotorConnectedAlert.set(!endEffectorConnected);
    m_coral_detector.robotPeriodic();
  }
}
