package frc.robot.subsystems.endEffector;

import dev.doglog.DogLog;
import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class EndEffectorSubsystem extends SubsystemBase {

  EndEffectorIO endEffectorIO;
  public final Trigger coralTriggered;

  public EndEffectorSubsystem() {

    if (RobotBase.isSimulation() || true) {
      endEffectorIO = new EndEffectorIOSim();
    } else {
      // endEffectorIO = new EndEffectorIOSparkMax();
      endEffectorIO = new EndEffectorIOTalon();
    }

    coralTriggered = new Trigger(() -> endEffectorIO.isSensorTriggered());
  }

  public Command setVoltage(double voltage) {
    return Commands.runOnce(() -> endEffectorIO.setVoltage(voltage));
  }

  public Command stopMotor() {
    return Commands.runOnce(() -> endEffectorIO.stopMotor());
  }

  @Override
  public void periodic() {
    double startTime = HALUtil.getFPGATime();

    endEffectorIO.update();
    DogLog.log("EndEffector/Velocity", endEffectorIO.getVoltage());
    DogLog.log("EndEffector/Voltage", endEffectorIO.getVelocity());

    DogLog.log("Loop Time/End Effector", (HALUtil.getFPGATime() - startTime) / 1000);
  }
}
