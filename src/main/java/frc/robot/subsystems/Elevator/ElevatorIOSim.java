package frc.robot.subsystems.Elevator;

import dev.doglog.DogLog;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim implements ElevatorIO {
  private ElevatorSim elevatorSim =
      new ElevatorSim(0.12, 0.01, DCMotor.getFalcon500Foc(2), 0, 1.7, true, 0);

  private Constraints constraints = new Constraints(300, 150);
  private ProfiledPIDController pidController = new ProfiledPIDController(.1, 0, 0, constraints);

  public void setPosition(double position) {
    pidController.setGoal(position);
  }

  public double getPosition() {
    return ElevatorSubsystem.metersToRotations(elevatorSim.getPositionMeters());
  }

  public void update() {
    elevatorSim.update(.020);

    double pidOutput = pidController.calculate(getPosition());

    DogLog.log("Elevator/Simulation/PID Output", pidOutput);

    elevatorSim.setInputVoltage(pidOutput);
  }

  public boolean getReverseLimit() {
    return elevatorSim.getPositionMeters() == 0;
  }

  public boolean getForwardLimit() {
    return elevatorSim.getPositionMeters() >= ElevatorConstants.TOP_METER;
  }

  @Override
  public void setVoltage(double voltage) {
    elevatorSim.setInputVoltage(voltage);
  }
}
