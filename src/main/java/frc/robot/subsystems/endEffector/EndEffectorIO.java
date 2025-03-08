package frc.robot.subsystems.endEffector;

interface EndEffectorIO {

  void setVoltage(double voltage);

  void stopMotor();

  public boolean coralLoaded();

  double getVelocity();

  double getVoltage();

  void setAmps(double current);

  void update();
}
