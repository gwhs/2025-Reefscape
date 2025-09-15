package frc.robot.subsystems.groundIntake;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

  public Command setAngleAndAmp(double pivotAngle, double amp, double dutyCycle) {
    // double piv =
    //     MathUtil.clamp(
    //         pivotAngle,
    //         GroundIntakeConstants.GROUND_INTAKE_LOWER_BOUND,
    //         GroundIntakeConstants.GROUND_INTAKE_UPPER_BOUND);
    return this.runOnce(
            () -> {
              groundintakeIO.resetPivotEncoder();
              groundintakeIO.setAngle(pivotAngle);
              groundintakeIO.runAmp(amp, dutyCycle);
            })
        .withName("Ground Intake: pivot angle: " + pivotAngle + " Intake Amp: " + amp);
  }

  public double getAngle() {
    return groundintakeIO.getPivotAngle();
  }

  @Override
  public void periodic() {
    groundintakeIO.update();
    DogLog.log("groundIntake/Pivot/angle", groundintakeIO.getPivotAngle());
  }

  public Command increaseAngle(double angle) {
    return Commands.runOnce(() -> groundintakeIO.setAngle(getAngle() - angle));
  }

  public Command decreaseAngle(double angle) {
    return Commands.runOnce(() -> groundintakeIO.setAngle(getAngle() + angle));
  }
}
