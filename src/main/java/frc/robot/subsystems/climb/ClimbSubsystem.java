package frc.robot.subsystems.climb;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

  public double getHeight() {
    return climbIO.getHeight();
  }

  public double setHeight(double desiredHeight) {
    return climbIO.setHeight();
  }
}
