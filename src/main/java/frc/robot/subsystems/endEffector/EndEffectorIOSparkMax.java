package frc.robot.subsystems.endEffector;


import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

class EndEffectorIOSparkMax implements EndEffectorIO {

  private SparkMax motor = new SparkMax(EndEffectorConstants.deviceID, MotorType.kBrushless);
  private RelativeEncoder encoder = motor.getEncoder();
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
      return encoder.getVelocity() / 60; // returns the RPM so div by 60 for RPS
  }

  @Override
  public double getVoltage() {
     return motor.getBusVoltage();
  }

  @Override
  public void update() {
  // logged in subsystem
  }
}
