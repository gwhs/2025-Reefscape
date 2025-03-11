package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import dev.doglog.DogLog;
import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class ArmSubsystem extends SubsystemBase {
  private ArmIO armIO;

  private final SysIdRoutine m_sysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Units.Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
              null, // Use default timeout (10 s)
              // Log state with Phoenix SignalLogger class
              (state) -> SignalLogger.writeString("state", state.toString())),
          new SysIdRoutine.Mechanism((volts) -> armIO.setVoltage(volts.in(Volts)), null, this));

  public ArmSubsystem() {
    if (RobotBase.isSimulation()) {
      armIO = new ArmIOSim();
    } else {
      armIO = new ArmIOReal();
    }
  }

  /**
   * drives the arm until it reaches the given provided angle
   *
   * @param angle Angle to drive the arm to in degrees
   */
  public Command setAngle(double angle) {
    double clampedAngle =
        MathUtil.clamp(angle, ArmConstants.ARM_LOWER_BOUND, ArmConstants.ARM_UPPER_BOUND);
    return this.runOnce(
            () -> {
              armIO.setAngle(clampedAngle);
            })
        .andThen(Commands.waitUntil(() -> MathUtil.isNear(clampedAngle, armIO.getPosition(), 1)));
  }

  @Override
  public void periodic() {
    double startTime = HALUtil.getFPGATime();

    armIO.update();
    DogLog.log("Arm/arm angle", armIO.getPosition());
    DogLog.log("Loop Time/Arm", (HALUtil.getFPGATime() - startTime) / 1000);
  }

  /**
   * @return the arm's angle
   */
  public double getAngle() {
    return armIO.getPosition();
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }

  /**
   * @param degrees the degrees to add
   * @return run the command
   */
  public Command increaseAngle(double degrees) {
    return Commands.runOnce(
            () -> {
              armIO.setAngle(armIO.getPosition() + degrees);
            })
        .andThen(Commands.waitUntil(() -> MathUtil.isNear(degrees, armIO.getPosition(), 1)));
  }

  /**
   * @param degrees the degrees to decrease to
   * @return run the command
   */
  public Command decreaseAngle(double degrees) {
    return Commands.runOnce(
            () -> {
              armIO.setAngle(armIO.getPosition() - degrees);
            })
        .andThen(Commands.waitUntil(() -> MathUtil.isNear(degrees, armIO.getPosition(), 1)));
  }
}
