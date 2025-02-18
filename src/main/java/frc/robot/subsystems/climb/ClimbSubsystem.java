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

    SmartDashboard.putData("Climb Command/extend", extend());
    SmartDashboard.putData("Climb Command/retract", retract());
  }

  @Override
  public void periodic() {
    climbIO.update();
    DogLog.log("Climb/Climb Position", getPosition());
  }

  public double getPosition() {
    return climbIO.getPosition();
  }

  public Command extend() {
    return this.runOnce(
            () -> {
              climbIO.setPosition(ClimbConstants.EXTEND_CLIMB_POSITION);
            })
        .andThen(Commands.waitUntil(() -> MathUtil.isNear(ClimbConstants.EXTEND_CLIMB_POSITION, climbIO.getPosition(), 0.1)));
  }

  public Command retract() {
    return this.runOnce(
            () -> {
              climbIO.setPosition(ClimbConstants.RETRACT_CLIMB_POSITION);
            })
        .andThen(Commands.waitUntil(() -> MathUtil.isNear(ClimbConstants.RETRACT_CLIMB_POSITION, climbIO.getPosition(), 0.1)));
  }
}
