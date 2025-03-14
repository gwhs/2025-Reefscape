package frc.robot.subsystems.arm;

public interface ArmIO {
  public void setAngle(double angle);

  public double getPosition();

  public void update();

  public void setVoltage(double volts);

  public void setEmergencyMode(boolean emergency);

  public double getPositionError();
}
