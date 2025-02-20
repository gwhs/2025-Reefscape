package frc.robot.subsystems.climb;

import dev.doglog.DogLog;
import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {
  private ClimbIO climbIO;

  public ClimbSubsystem() {
    if (RobotBase.isSimulation() || true) {
      climbIO = new ClimbIOSim();
    } else {
      climbIO = new ClimbIOReal();
    }
  }

  @Override
  public void periodic() {
    double startTime = HALUtil.getFPGATime();

    climbIO.update();

    DogLog.log("Loop Time/Climb", (HALUtil.getFPGATime() - startTime) / 1000);
  }

  public double getPosition() {
    return climbIO.getPosition();
  }

  public Command setPosition(double desiredPos) {
    return this.runOnce(
            () -> {
              climbIO.setPosition(desiredPos);
            })
        .andThen(Commands.waitUntil(() -> MathUtil.isNear(desiredPos, climbIO.getPosition(), 0.1)));
  }
}
