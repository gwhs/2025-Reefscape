package frc.robot.subsystems.endEffector;

interface EndEffectorIO {

  void setVoltage(double voltage);

  void stopMotor();

  double getVelocity();

  double getVoltage();

  void update();
}
