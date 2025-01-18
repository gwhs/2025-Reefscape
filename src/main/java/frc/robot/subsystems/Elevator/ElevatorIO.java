// Copied from
// https://github.com/gwhs/2024v2/blob/2025-beta/src/main/java/frc/robot/subsystems/ElevatorSubsystem/ElevatorIO.java
package frc.robot.subsystems.Elevator;

public interface ElevatorIO {

  boolean[] metersToRotations = null;

  public void setPosition(double position);

  public double getPosition();

  public void update();

  public void setVoltage(double voltage);

  public boolean getReverseLimit();

  public boolean getForwardLimit();
}
