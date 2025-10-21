package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import dev.doglog.DogLog;

public class ElevatorIOReal implements ElevatorIO {

  private boolean EmergencyMode;
  private TalonFX m_FrontMotor =
      new TalonFX(ElevatorConstants.FrontElevatorMotorID, ElevatorConstants.ElevatorMotorCanBus);
  private TalonFX m_BackMotor =
      new TalonFX(ElevatorConstants.BackElevatorMotorID, ElevatorConstants.ElevatorMotorCanBus);
  private double FrontMotorPos = m_FrontMotor.getRotorPosition().getValueAsDouble();
  private double BackMotorPos = m_FrontMotor.getRotorPosition().getValueAsDouble();
  private final StatusSignal<ForwardLimitValue> forwardLimit = m_FrontMotor.getForwardLimit();
  private final StatusSignal<ReverseLimitValue> reverseLimit = m_FrontMotor.getReverseLimit();
  private final StatusSignal<Double> frontElevatorMotorPIDGoal =
      m_FrontMotor.getClosedLoopReference();

  public ElevatorIOReal() {}

  @Override
  public double getRotation() {
    return FrontMotorPos;
  }

  @Override
  public void setRotation(double rotation) {
    m_FrontMotor.set(rotation);
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

  public void setPosition(double newValue) {
    if (EmergencyMode == false) {
      m_FrontMotor.setPosition(newValue);
      m_BackMotor.setPosition(newValue);
    } else {
      m_FrontMotor.stopMotor();
      m_FrontMotor.stopMotor();
    }
  }

  @Override
  public void setVoltage(double voltage) {
    if (!EmergencyMode) {
      m_FrontMotor.setVoltage(voltage);
      m_BackMotor.setVoltage(voltage);
    } else {
      m_FrontMotor.stopMotor();
      m_BackMotor.stopMotor();
    }
  }

  public boolean getForwardLimit() {
    return forwardLimit.getValue().value == 0;
  }

  public boolean getReverseLimit() {
    return reverseLimit.getValue().value == 0;
  }

  public void setNeutralMode(NeutralModeValue mode) {
    m_FrontMotor.setNeutralMode(mode);
    m_BackMotor.setNeutralMode(mode);
  }

  public double getPIDGoalRotation() {
    return frontElevatorMotorPIDGoal.getValueAsDouble();
  }
}
