package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import dev.doglog.DogLog;

public class ElevatorIOReal implements ElevatorIO {

  private boolean EmergencyMode;
  private final TalonFX FrontMotor =
      new TalonFX(
          ElevatorConstants.FRONT_ELEVATOR_MOTOR_ID, ElevatorConstants.ELEVATOR_MOTOR_CAN_BUS);
  private final TalonFX BackMotor =
      new TalonFX(
          ElevatorConstants.BACK_ELEVATOR_MOTOR_ID, ElevatorConstants.ELEVATOR_MOTOR_CAN_BUS);
  private final StatusSignal<ForwardLimitValue> ForwardLimit = FrontMotor.getForwardLimit();
  private final StatusSignal<ReverseLimitValue> ReverseLimit = FrontMotor.getReverseLimit();
  private final StatusSignal<Double> FrontElevatorMotorPIDGoal =
      FrontMotor.getClosedLoopReference();
  private double BackMotorPos = FrontMotor.getRotorPosition().getValueAsDouble();
  private double FrontMotorPos = FrontMotor.getRotorPosition().getValueAsDouble();

  public ElevatorIOReal() {}

  @Override
  public double getRotation() {
    return FrontMotorPos;
  }

  @Override
  public void setRotation(double rotation) {
    FrontMotor.set(rotation);
  }

  @Override
  public void update() {

    DogLog.log("Elevator/FrontMotorPos", FrontMotorPos);
    DogLog.log("Elevator/BackMotorPos", BackMotorPos);
  }

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
      FrontMotor.setPosition(newValue);
      BackMotor.setPosition(newValue);
    } else {
      FrontMotor.stopMotor();
      BackMotor.stopMotor();
    }
  }

  @Override
  public void setVoltage(double voltage) {
    if (!EmergencyMode) {
      FrontMotor.setVoltage(voltage);
      BackMotor.setVoltage(voltage);
    } else {
      FrontMotor.stopMotor();
      BackMotor.stopMotor();
    }
  }

  @Override
  public boolean getForwardLimit() {
    return ForwardLimit.getValue().value == 0;
  }

  @Override
  public boolean getReverseLimit() {
    return ReverseLimit.getValue().value == 0;
  }

  @Override
  public void setNeutralMode(NeutralModeValue mode) {
    FrontMotor.setNeutralMode(mode);
    BackMotor.setNeutralMode(mode);
  }

  @Override
  public double getPIDGoalRotation() {
    return FrontElevatorMotorPIDGoal.getValueAsDouble();
  }
}
