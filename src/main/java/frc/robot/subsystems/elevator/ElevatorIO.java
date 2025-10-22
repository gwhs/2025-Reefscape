package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.signals.NeutralModeValue;

public interface ElevatorIO {
  public void runPosition(double newValue);

  public void runRotation(double rotation);

  public double getRotation();

  public void update();

  public void setVoltage(double voltage);

  public boolean getReverseLimit();

  public boolean getForwardLimit();

  public void setNeutralMode(NeutralModeValue mode);

  public void setEmergencyMode(boolean emergency);

  public boolean getEmergencyMode();

  public double getPIDGoalRotation();
}
