// Copied from
// https://github.com/gwhs/2024v2/blob/2025-beta/src/main/java/frc/robot/subsystems/ElevatorSubsystem/ElevatorIO.java
package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.signals.NeutralModeValue;

public interface ElevatorIO {

  boolean[] metersToRotations = null;

  public void setPosition(double newValue);

  public void setRotation(double rotation);

  public double getRotation();

  public void update();

  public void setVoltage(double voltage);

  public boolean getReverseLimit();

  public boolean getForwardLimit();

  public void setNeutralMode(NeutralModeValue mode);

  public void setEmergencyMode(boolean emergency);
}
