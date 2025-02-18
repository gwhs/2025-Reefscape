package frc.robot.subsystems.climb;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.subsystems.arm.ArmConstants;

public class ClimbIOSim implements ClimbIO {

 private SingleJointedArmSim climbSim =
      new SingleJointedArmSim(
          DCMotor.getFalcon500Foc(1),
          ClimbConstants.CLIMB_GEAR_RATIO,
          0.1,
          0.1,
          Units.degreesToRadians(0),
          Units.degreesToRadians(300),
          false,
          Units.degreesToRadians(90));

  public ClimbIOSim() {}

  private TrapezoidProfile.Constraints constraints =
      new TrapezoidProfile.Constraints(
          ClimbConstants.MAX_VELOCITY * 360 / 60, ClimbConstants.MAX_ACCELERATION * 360 / 60);
  private ProfiledPIDController pidController = new ProfiledPIDController(.1, 0, 0, constraints);

  @Override
  public double getPosition() {
    return Units.radiansToDegrees(climbSim.getAngleRads());
  }

  @Override
  public void setPosition(double angle) {
    pidController.setGoal(angle);
  }

  @Override
  public void update() {
    climbSim.update(0.20);

    double pidOutput = pidController.calculate(getPosition());

    climbSim.setInputVoltage(pidOutput);
  }
}
