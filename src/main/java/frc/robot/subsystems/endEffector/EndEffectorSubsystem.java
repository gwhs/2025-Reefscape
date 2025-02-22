package frc.robot.subsystems.endEffector;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class EndEffectorSubsystem extends SubsystemBase {

  EndEffectorIO endEffectorIO;
  public final Trigger coralTriggered;

  public EndEffectorSubsystem() {

    if (RobotBase.isSimulation()) {
      endEffectorIO = new EndEffectorIOSim();
    } else {
      // endEffectorIO = new EndEffectorIOSparkMax();
      endEffectorIO = new EndEffectorIOTalon();
    }

    coralTriggered = new Trigger(() -> endEffectorIO.isSensorTriggered());

    SmartDashboard.putData("End Effector Command/End Effector Shoot", shoot());
    SmartDashboard.putData("End Effector Command/End Effector Intake", intake());
    SmartDashboard.putData("End Effector Command/Stop End Effector", stopMotor());
  }

  public Command setVoltage(double voltage) {
    return Commands.runOnce(() -> endEffectorIO.setVoltage(voltage));
  }

  public Command shoot() {
    return Commands.runOnce(() -> endEffectorIO.setVoltage(-12));
  }

  public Command intake() {
    return Commands.runOnce(() -> endEffectorIO.setVoltage(6));
  }

  public Command stopMotor() {
    return Commands.runOnce(() -> endEffectorIO.stopMotor());
  }

  @Override
  public void periodic() {
    endEffectorIO.update();
    DogLog.log("EndEffector/Voltage", endEffectorIO.getVoltage());
    DogLog.log("EndEffector/Velocity", endEffectorIO.getVelocity());
  }
}
