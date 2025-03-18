package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim implements ElevatorIO {
  private ElevatorSim elevatorSim =
      new ElevatorSim(0.12, 0.01, DCMotor.getFalcon500Foc(2), 0, 1.7, true, 0);

  private Constraints constraints =
      new Constraints(ElevatorConstants.MAX_VELOCITY, ElevatorConstants.MAX_ACCELERATION);
  private ProfiledPIDController pidController = new ProfiledPIDController(.1, 0, 0, constraints);

  public void setRotation(double rotation) {
    pidController.setGoal(rotation);
  }

  public double getRotation() {
    return ElevatorSubsystem.metersToRotations(elevatorSim.getPositionMeters());
  }

  public void update() {
    elevatorSim.update(.020);

    double pidOutput = pidController.calculate(getRotation());

    DogLog.log("Elevator/Simulation/PID Output", pidOutput);

    elevatorSim.setInputVoltage(pidOutput);
  }

  private boolean m_emergencyMode;

  public boolean getReverseLimit() {
    return elevatorSim.getPositionMeters() == 0;
  }

  public boolean getForwardLimit() {
    return elevatorSim.getPositionMeters() >= ElevatorConstants.TOP_METER;
  }

  public void setNeutralMode(NeutralModeValue mode) {
    DogLog.log("Elevator/Simulation/NeutralMode", mode);
  }

  @Override
  public void setVoltage(double voltage) {
    elevatorSim.setInputVoltage(voltage);
  }

  @Override
  public void setPosition(double newValue) {
    elevatorSim.setState(newValue, 0);
  }

  @Override
  public void setEmergencyMode(boolean emergency) {
    m_emergencyMode = emergency;
    setVoltage(0);
  }
}
