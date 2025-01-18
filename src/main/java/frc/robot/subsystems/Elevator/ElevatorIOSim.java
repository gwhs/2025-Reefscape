package frc.robot.subsystems.Elevator;

import dev.doglog.DogLog;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim implements ElevatorIO {
  private ElevatorSim elevatorSim =
      new ElevatorSim(0.12, 0.01, DCMotor.getFalcon500Foc(2), 0, 50, true, 0);

  private Constraints constraints =
      new Constraints(ElevatorConstants.MAX_VELOCITY, ElevatorConstants.MAX_ACCELERATION);
  private ProfiledPIDController pidController =
      new ProfiledPIDController(
          .25, ElevatorConstants.ELEVATOR_PID_KI, ElevatorConstants.ELEVATOR_PID_KD, constraints);

  public void setPosition(double position) {
    pidController.setGoal(ElevatorSubsystem.rotationsToMeters(position));
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

  @Override
  public void setVoltage(double voltage) {
    elevatorSim.setInputVoltage(voltage);
  }
}
