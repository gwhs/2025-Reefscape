package frc.robot.subsystems.arm;

public interface ArmIO {
  public void setAngle(double angle);

  public double getPosition();

  public void update();

  public void setVoltage(double volts);

  public void setEmergencyMode(
      boolean emergency); // make so if emergency mode = true, set volts to 0, same w/ elevator
}
