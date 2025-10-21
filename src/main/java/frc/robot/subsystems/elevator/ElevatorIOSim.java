package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim implements ElevatorIO {
  private boolean EmergencyMode;

  private ElevatorSim ElevatorSim =
      new ElevatorSim(0.12, 0.01, DCMotor.getFalcon500Foc(2), 0, 1.7, true, 0);
  private Constraints Constraints =
      new Constraints(ElevatorConstants.MAX_VELOCITY, ElevatorConstants.MAX_ACCELERATION);
  private ProfiledPIDController PIDController = new ProfiledPIDController(.1, 0, 0, Constraints);

  public ElevatorIOSim() {}

  @Override
  public double getRotation() {
    return ElevatorSubsystem.metersToRotations(ElevatorSim.getPositionMeters());
  }

  @Override
  public void setRotation(double rotation) {
    PIDController.setGoal(rotation);
  }

  @Override
  public void update() {}

  @Override
  public void setEmergencyMode(boolean emergency) {
    EmergencyMode = emergency;
  }

  @Override
  public boolean getEmergencyMode() {
    return EmergencyMode;
  }

  @Override
  public void setPosition(double newValue) {
    if (!EmergencyMode) {
      ElevatorSim.setState(newValue, 0);
    }
  }

  @Override
  public void setVoltage(double voltage) {
    if (!EmergencyMode) {
      ElevatorSim.setInputVoltage(voltage);
    } else {
      ElevatorSim.setInputVoltage(voltage);
    }
  }

  @Override
  public boolean getReverseLimit() {
    return ElevatorSim.getPositionMeters() == 0;
  }

  @Override
  public boolean getForwardLimit() {
    return ElevatorSim.getPositionMeters() >= ElevatorConstants.TOP_METER;
  }

  @Override
  public double getPIDGoalRotation() {
    return PIDController.getGoal().position;
  }

  @Override
  public void setNeutralMode(NeutralModeValue mode) {
    DogLog.log("Elevator/Simulation/NeutralMode", mode);
  }
}
