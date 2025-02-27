package frc.robot.subsystems.endEffector;

interface EndEffectorIO {

  void setVoltage(double voltage);

  void stopMotor();

  public boolean isSensorTriggered();

  double getVelocity();

  double getVoltage();

  void setControl(double current);

  void update();
}
