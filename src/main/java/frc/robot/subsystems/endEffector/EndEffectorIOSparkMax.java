package frc.robot.subsystems.endEffector;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import dev.doglog.DogLog;

class EndEffectorIOSparkMax implements EndEffectorIO {

    private SparkMax motor = new SparkMax(EndEffectorConstants.deviceID, MotorType.kBrushless);

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
      DogLog.log("EndEffector/Voltage", motor.getBusVoltage());
      DogLog.log("EndEffector/Temperature", motor.getMotorTemperature());
  }
}
