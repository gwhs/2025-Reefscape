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
    if (RobotBase.isSimulation()) {
      climbIO = new ClimbIOSim();
    } else {
      climbIO = new ClimbIOReal();
    }
  }

  @Override
  public void periodic() {
    double startTime = HALUtil.getFPGATime();

    climbIO.update();
    DogLog.log("Climb/Climb Position", getPosition());

    DogLog.log("Loop Time/Climb", (HALUtil.getFPGATime() - startTime) / 1000);
  }

  /**
   * @return the climb's position
   */
  public double getPosition() {
    return climbIO.getPosition();
  }

  /**
   * @return extend the climb NOTE: see ClimbConstants for the position
   */
  public Command climb() {
    return this.runOnce(
            () -> {
              climbIO.setPosition(ClimbConstants.CLIMB_CLIMB_POSITION);
            })
        .andThen(
            Commands.waitUntil(
                () ->
                    MathUtil.isNear(
                        ClimbConstants.CLIMB_CLIMB_POSITION, climbIO.getPosition(), 2)));
  }

  /**
   * @return retract the climb NOTE: see ClimbConstants for the position
   */
  public Command stow() {
    return this.runOnce(
            () -> {
              climbIO.setPosition(ClimbConstants.STOW_CLIMB_POSITION);
            })
        .andThen(
            Commands.waitUntil(
                () ->
                    MathUtil.isNear(ClimbConstants.STOW_CLIMB_POSITION, climbIO.getPosition(), 2)));
  }

  public Command latch() {
    return this.runOnce(
            () -> {
              climbIO.setPosition(ClimbConstants.LATCH_CLIMB_POSITION);
            })
        .andThen(
            Commands.waitUntil(
                () ->
                    MathUtil.isNear(
                        ClimbConstants.LATCH_CLIMB_POSITION, climbIO.getPosition(), 2)));
  }
}
