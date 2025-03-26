package frc.robot.subsystems.groundIntake;

import dev.doglog.DogLog;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class GroundIntakeIOSim implements GroundIntakeIO {

  private SingleJointedArmSim pivotMotorSim =
      new SingleJointedArmSim(
          DCMotor.getFalcon500Foc(1),
          GroundIntakeConstants.PIVOT_GEAR_RATIO,
          0.1,
          1,
          Units.degreesToRadians(GroundIntakeConstants.GROUND_INTAKE_LOWER_BOUND),
          Units.degreesToRadians(GroundIntakeConstants.GROUND_INTAKE_UPPER_BOUND),
          false,
          Units.degreesToRadians(90));

  private TrapezoidProfile.Constraints constraints =
      new TrapezoidProfile.Constraints(
          GroundIntakeConstants.MAX_VELOCITY * 360, GroundIntakeConstants.MAX_ACCELERATION * 360);
  private ProfiledPIDController pidController = new ProfiledPIDController(.1, 0, 0, constraints);

  private FlywheelSim spinMotorSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              DCMotor.getFalcon500Foc(1), 0.0001, GroundIntakeConstants.SPIN_GEAR_RATIO),
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
  public void setAngle(double angle) {
    pidController.setGoal(angle);
  }

  @Override
  public double getPivotAngle() {
    return Units.radiansToDegrees(pivotMotorSim.getAngleRads());
  }

  @Override
  public void update() {
    spinMotorSim.update(0.20);
    pivotMotorSim.update(0.20);

    double pidOutput = pidController.calculate(getPivotAngle());

    pivotMotorSim.setInputVoltage(pidOutput);

    DogLog.log("groundIntake/Spin/voltage", spinMotorSim.getInputVoltage());
    DogLog.log("groundIntake/Pivot/position", Units.radiansToDegrees(pivotMotorSim.getAngleRads()));
  }

  @Override
  public void resetPivotEncoder() {}
}
