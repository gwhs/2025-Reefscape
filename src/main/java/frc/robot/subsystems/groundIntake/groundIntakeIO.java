package frc.robot.subsystems.groundIntake;

public interface GroundIntakeIO {

  public void setPivotMotorVoltage(double voltage);

  public void setSpinMotorVoltage(double voltage);

  public void setAngle(double angle);

  public void update();
}
