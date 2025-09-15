package frc.robot.subsystems.groundIntake;

public class GroundIntakeIODisabled implements GroundIntakeIO {

  @Override
  public void setAngle(double angle) {}

  @Override
  public void setSpinMotorVoltage(double voltage) {}

  @Override
  public void setPivotMotorVoltage(double voltage) {}

  @Override
  public double getPivotAngle() {
    return 0;
  }

  @Override
  public void resetPivotEncoder() {}

  @Override
  public void update() {}

  @Override
  public void runAmp(double amp, double dutyCycle) {}
}
