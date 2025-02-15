package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {
  private ClimbIO climbIO;

  public ClimbSubsystem() {
    if (RobotBase.isSimulation()) {
      climbIO = new ClimbIOSim();
    } else {
      climbIO = new ClimbIOReal();
    }
  }

  @Override
  public void periodic() {
    climbIO.update();
  }

  public double getPosition() {
    return climbIO.getPosition();
  }

  public void setPosition(double desiredPos) {
    climbIO.setPosition(desiredPos);
  }
}
