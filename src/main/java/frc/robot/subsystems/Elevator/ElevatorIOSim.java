package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ElevatorIOSim implements ElevatorIO {
  private DCMotorSim rightMotorSim = new DCMotorSim(
      LinearSystemId.createDCMotorSystem(DCMotor.getFalcon500(1), 0.0001, 1),DCMotor.getFalcon500(1));
  private DCMotorSim leftMotorSim = new DCMotorSim(
      LinearSystemId.createDCMotorSystem(DCMotor.getFalcon500(1), 0.0001, 1),
      DCMotor.getFalcon500(1));
  private Constraints constraints = new Constraints(ElevatorConstant.MAX_VELOCITY, ElevatorConstant.MAX_ACCELERATION);
  private ProfiledPIDController leftpidController = new ProfiledPIDController(ElevatorConstant.ELEVATOR_PID_KP,
      ElevatorConstant.ELEVATOR_PID_KI, ElevatorConstant.ELEVATOR_PID_KD, constraints);
  private ProfiledPIDController rightpidController = new ProfiledPIDController(ElevatorConstant.ELEVATOR_PID_KP,
      ElevatorConstant.ELEVATOR_PID_KI, ElevatorConstant.ELEVATOR_PID_KD, constraints);

  @Override
  public void setPositionLeft(double position) {
      leftpidController.setGoal(position);
  }

  public void setPositionRight(double position) {
      rightpidController.setGoal(position);
  }

  @Override
  public double getRightMotorPosition() {
    return rightMotorSim.getAngularPositionRotations();
  }

  @Override
  public double getLeftMotorPosition() {
    return leftMotorSim.getAngularPositionRotations();
  }

  @Override
  public void setLeftMotorSpeed(double speed) {
    leftMotorSim.setInputVoltage(speed * 10);
  }

  @Override
  public void setRightMotorSpeed(double speed) {
    rightMotorSim.setInputVoltage(speed * 10);
  }

  public void update() {
    rightMotorSim.update(.020);
    leftMotorSim.update(.020);

    double pidOutputLeft = leftpidController.calculate(getLeftMotorPosition());
    double pidOutputRight = rightpidController.calculate(getRightMotorPosition());

    setLeftMotorSpeed(pidOutputLeft);
    setRightMotorSpeed(pidOutputRight);

  }
}
