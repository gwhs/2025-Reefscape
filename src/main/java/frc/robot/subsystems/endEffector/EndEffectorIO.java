package frc.robot.subsystems.endEffector;

import edu.wpi.first.wpilibj2.command.Command;

interface EndEffectorIO {

  Command setVoltage(double voltage);

  Command stopMotor();

  void update();
}
