package frc.robot.subsystems.endEffector;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

class EndEffectorIOSparkMax implements EndEffectorIO {

  private SparkMax motor = new SparkMax(EndEffectorConstants.deviceID, MotorType.kBrushless);

  @Override
  public Command setVoltage(double voltage) {

   return Commands.runOnce( () -> motor.setVoltage(voltage));
  }

  @Override
  public Command stopMotor() {
   return Commands.runOnce( () -> motor.setVoltage(0));
  }

  @Override
  public void update() {
    DogLog.log("EndEffector/Voltage", motor.getBusVoltage());
    DogLog.log("EndEffector/Temperature", motor.getMotorTemperature());
  }
}
