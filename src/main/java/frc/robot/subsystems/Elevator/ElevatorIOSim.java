package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim implements ElevatorIO {
  private ElevatorSim elevatorSim =
      new ElevatorSim(0, 0, DCMotor.getFalcon500Foc(2), 0, 50, true, 0);

  private Constraints constraints =
      new Constraints(ElevatorConstants.MAX_VELOCITY, ElevatorConstants.MAX_ACCELERATION);
  private ProfiledPIDController pidController =
      new ProfiledPIDController(
          ElevatorConstants.ELEVATOR_PID_KP,
          ElevatorConstants.ELEVATOR_PID_KI,
          ElevatorConstants.ELEVATOR_PID_KD,
          constraints);

  public void setPosition(double position) {}

  public double getPosition() {
    return 0;
  }

  public void update() {
    elevatorSim.update(.020);

    // double pidOutputLeft = leftpidController.calculate(getLeftMotorPosition());
    // double pidOutputRight = rightpidController.calculate(getRightMotorPosition());

    // setLeftMotorSpeed(pidOutputLeft);
    // setRightMotorSpeed(pidOutputRight);

  }
}
