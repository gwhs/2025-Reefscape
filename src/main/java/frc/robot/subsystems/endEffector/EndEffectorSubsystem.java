package frc.robot.subsystems.endEffector;

import dev.doglog.DogLog;
import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EndEffectorSubsystem extends SubsystemBase {

  private EndEffectorIO endEffectorIO;

  // public final Trigger coralTriggered;

  /**
   * there are two implemenations for talon motors and sparkmax motors we are probably going to use
   * talon tho
   */
  public EndEffectorSubsystem() {

    if (RobotBase.isSimulation()) {
      endEffectorIO = new EndEffectorIOSim();
    } else {
      // endEffectorIO = new EndEffectorIOSparkMax();
      endEffectorIO = new EndEffectorIOTalon();
    }

    // coralTriggered = new Trigger(() -> endEffectorIO.coralLoaded());
  }

  /**
   * @param voltage the voltage to set to
   * @return set the motor to the voltage
   */
  public boolean coralLoaded() {
    return endEffectorIO.coralLoaded();
  }

  public Command setVoltage(double voltage) {
    return Commands.runOnce(() -> endEffectorIO.setVoltage(voltage));
  }

  public Command shoot() {
    return Commands.runOnce(() -> endEffectorIO.setVoltage(-10));
  }

  public Command intake() {
    return Commands.runOnce(
        () -> endEffectorIO.setAmps(EndEffectorConstants.INTAKE_CORAL_CURRENT, 0.35));
  }

  public Command holdCoral() {
    return Commands.runOnce(
        () -> endEffectorIO.setAmps(EndEffectorConstants.HOLD_CORAL_CURRENT, 0.2));
  }

  /**
   * @return stop the motor
   */
  public Command stopMotor() {
    return Commands.runOnce(() -> endEffectorIO.stopMotor());
  }

  @Override
  public void periodic() {
    double startTime = HALUtil.getFPGATime();

    endEffectorIO.update();
    DogLog.log("EndEffector/Voltage", endEffectorIO.getVoltage());
    DogLog.log("EndEffector/Velocity", endEffectorIO.getVelocity());

    DogLog.log("Loop Time/End Effector", (HALUtil.getFPGATime() - startTime) / 1000);
  }
}
