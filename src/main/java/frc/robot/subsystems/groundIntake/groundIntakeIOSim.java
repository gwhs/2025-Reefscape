package frc.robot.subsystems.groundIntake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class groundIntakeIOSim implements groundIntakeIO {

  private SingleJointedArmSim pivotMotorSim =
      new SingleJointedArmSim(
          DCMotor.getFalcon500Foc(1),
          groundIntakeConstants.PIVOT_GEAR_RATIO,
          0.1,
          1,
          Units.degreesToRadians(groundIntakeConstants.GROUND_INTAKE_LOWER_BOUND),
          Units.degreesToRadians(groundIntakeConstants.GROUND_INTAKE_UPPER_BOUND),
          false,
          Units.degreesToRadians(90));

  private FlywheelSim spinMotorSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              DCMotor.getFalcon500Foc(1), 0.0001, groundIntakeConstants.SPIN_GEAR_RATIO),
          DCMotor.getFalcon500Foc(1));

  @Override
  public void setPivotMotorVoltage(double voltage) {
    pivotMotorSim.setInputVoltage(voltage);
  }

  @Override
  public void setSpinMotorVoltage(double voltage) {
    spinMotorSim.setInputVoltage(voltage);
  }

  @Override
  public void update() {
    spinMotorSim.update(0.20);
    pivotMotorSim.update(0.20);
  }
}
