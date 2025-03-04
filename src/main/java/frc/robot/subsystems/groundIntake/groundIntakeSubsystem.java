package frc.robot.subsystems.groundIntake;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GroundIntakeSubsystem extends SubsystemBase {

  private final GroundIntakeIO groundintakeIO;

  public GroundIntakeSubsystem() {
    if (RobotBase.isSimulation()) {
      groundintakeIO = new GroundIntakeIOSim();
    } else {
      groundintakeIO = new GroundIntakeIOReal();
    }
  }

  public Command setAngleAndVoltage(double pivotAngle, double voltage) {
    return this.runOnce(
            () -> {
              groundintakeIO.setAngle(pivotAngle);
              groundintakeIO.setSpinMotorVoltage(pivotAngle);
            })
        .withName("Ground Intake: pivot angle: " + pivotAngle + " Intake Voltage: " + voltage);
  }

  @Override
  public void periodic() {
    groundintakeIO.update();
  }
}
