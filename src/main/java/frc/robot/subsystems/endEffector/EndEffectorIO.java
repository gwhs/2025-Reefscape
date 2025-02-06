package frc.robot.subsystems.endEffector;



interface EndEffectorIO {

  void setVoltage(double voltage);

  void stopMotor();

  void update();
}
