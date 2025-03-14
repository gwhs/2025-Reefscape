package frc.robot.subsystems.groundIntake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    SmartDashboard.putData("groundIntake/Pivot/setAngle", setAngleAndVoltage(40, 50));
  }

  public Command setAngleAndVoltage(double pivotAngle, double voltage) {
    double volt = MathUtil.clamp(voltage, -12, 12);
    double piv =
        MathUtil.clamp(
            pivotAngle,
            GroundIntakeConstants.GROUND_INTAKE_LOWER_BOUND,
            GroundIntakeConstants.GROUND_INTAKE_UPPER_BOUND);
    return this.runOnce(
            () -> {
              groundintakeIO.resetPivotEncoder();
              groundintakeIO.setAngle(piv);
              groundintakeIO.setSpinMotorVoltage(volt);
            })
        .withName("Ground Intake: pivot angle: " + pivotAngle + " Intake Voltage: " + voltage);
  }

  public double getAngle() {
    return groundintakeIO.getPivotAngle();
  }

  @Override
  public void periodic() {
    groundintakeIO.update();
  }
}
