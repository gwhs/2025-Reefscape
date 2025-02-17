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

public class ClimbIOSim implements ClimbIO {

  private static LinearSystem<N2, N1, N2> plant = LinearSystemId.createDCMotorSystem(0, 1);

  public DCMotorSim climbSim =
      new DCMotorSim(plant, DCMotor.getFalcon500(1), ClimbConstants.CLIMB_MEASUREMENT_STDEV, 1);

  public ClimbIOSim() {}

  private TrapezoidProfile.Constraints constraints =
      new TrapezoidProfile.Constraints(
          ClimbConstants.MAX_VELOCITY * 360 / 60, ClimbConstants.MAX_ACCELERATION * 360 / 60);
  private ProfiledPIDController pidController = new ProfiledPIDController(.1, 0, 0, constraints);

  @Override
  public double getPosition() {
    return Units.radiansToDegrees(climbSim.getAngularPositionRad());
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
