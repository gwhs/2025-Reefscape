package frc.robot.subsystems.groundIntake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class groundIntakeIOReal implements groundIntakeIO {

  TalonFX spinMotor = new TalonFX(0, "rio");
  TalonFX pivotMotor = new TalonFX(0, "rio");
  StatusSignal<Voltage> spinMotorVoltage = spinMotor.getMotorVoltage();
  StatusSignal<Voltage> pivotMotorVoltage = pivotMotor.getMotorVoltage();
  StatusSignal<Temperature> spinMotorTemperature = spinMotor.getDeviceTemp();
  StatusSignal<Temperature> pivotMotorTemperature = pivotMotor.getDeviceTemp();
  StatusSignal<Current> spinMotorStatorCurrent = spinMotor.getStatorCurrent();
  StatusSignal<Current> pivotMotorStatorCurrent = pivotMotor.getStatorCurrent();

  public groundIntakeIOReal() {}

  @Override
  public void setSpinMotorVoltage(double voltage) {
    spinMotor.setVoltage(voltage);
  }

  @Override
  public void setPivotMotorVoltage(double voltage) {
    pivotMotor.setVoltage(voltage);
  }


  @Override
  public void update() {
    boolean groundIntakeisConnected =
        (BaseStatusSignal.refreshAll(
                spinMotorVoltage,
                pivotMotorVoltage,
                pivotMotorTemperature,
                spinMotorTemperature,
                pivotMotorStatorCurrent,
                spinMotorStatorCurrent))
            .isOK();
    DogLog.log("groundIntake/Spin/voltage", spinMotorVoltage.getValueAsDouble());
    DogLog.log("groundIntake/Pivot/voltage", pivotMotorVoltage.getValueAsDouble());
    DogLog.log("groundIntake/Spin/temperature", spinMotorTemperature.getValueAsDouble());
    DogLog.log("groundIntake/Pivot/temperature", pivotMotorTemperature.getValueAsDouble());
    DogLog.log("groundIntake/Spin/statorCurrent", spinMotorStatorCurrent.getValueAsDouble());
    DogLog.log("groundIntake/Pivot/statorCurrent", pivotMotorStatorCurrent.getValueAsDouble());
    DogLog.log("groundIntake/Connected", groundIntakeisConnected);
  }
}
