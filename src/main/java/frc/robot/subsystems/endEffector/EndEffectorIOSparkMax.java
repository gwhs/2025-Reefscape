package frc.robot.subsystems.endEffector;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.I2C;

class EndEffectorIOSparkMax implements EndEffectorIO {

  private SparkMax motor = new SparkMax(EndEffectorConstants.deviceID, MotorType.kBrushless);
  private RelativeEncoder encoder = motor.getEncoder();
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

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
    DogLog.log("endEncoder/Temperature", motor.getMotorTemperature());
  }

  @Override
  public boolean isSensorTriggered() {
    double distance = m_colorSensor.getProximity();
    if (distance > 1500) {
       return true; 
    } else {
     return false;
    }
   }
}
