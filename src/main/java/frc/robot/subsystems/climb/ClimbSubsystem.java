package frc.robot.subsystems.climb;

import dev.doglog.DogLog;
import edu.wpi.first.hal.HALUtil;
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

    SmartDashboard.putData("Climb Command/extend", climb());
    SmartDashboard.putData("Climb Command/retract", stow());
    SmartDashboard.putData("Climb Command/retract", latch());
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
              climbIO.setPosition(ClimbConstants.EXTEND_CLIMB_POSITION);
            })
        .andThen(
            Commands.waitUntil(
                () ->
                    MathUtil.isNear(
                        ClimbConstants.EXTEND_CLIMB_POSITION, climbIO.getPosition(), 0.1)));
  }

  /**
   * @return retract the climb NOTE: see ClimbConstants for the position
   */
  public Command stow() {
    return this.runOnce(
            () -> {
              climbIO.setPosition(ClimbConstants.RETRACT_CLIMB_POSITION);
            })
        .andThen(
            Commands.waitUntil(
                () ->
                    MathUtil.isNear(
                        ClimbConstants.RETRACT_CLIMB_POSITION, climbIO.getPosition(), 0.1)));
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
                        ClimbConstants.LATCH_CLIMB_POSITION, climbIO.getPosition(), 0.1)));
  }
}
