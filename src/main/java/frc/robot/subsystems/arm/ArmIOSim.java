package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ArmIOSim implements ArmIO {
  private SingleJointedArmSim armSim =
      new SingleJointedArmSim(
          DCMotor.getFalcon500Foc(1),
          ArmConstants.ARM_GEAR_RATIO,
          0.1,
          1,
          Units.degreesToRadians(ArmConstants.ARM_LOWER_BOUND),
          Units.degreesToRadians(ArmConstants.ARM_UPPER_BOUND),
          false,
          Units.degreesToRadians(90));

  private TrapezoidProfile.Constraints constraints =
      new TrapezoidProfile.Constraints(
          ArmConstants.MAX_VELOCITY * 360, ArmConstants.MAX_ACCELERATION * 360);
  private ProfiledPIDController pidController = new ProfiledPIDController(.1, 0, 0, constraints);

  private boolean m_emergencyMode;

  public ArmIOSim() {
    pidController.setGoal(90);
  }

  public double getPosition() {
    return Units.radiansToDegrees(armSim.getAngleRads());
  }

  @Override
  public void setAngle(double angle) {
    if (m_emergencyMode == false) {
      pidController.setGoal(angle);
    }
  }

  public double getPositionError() {
    return pidController.getPositionError();
  }

  /**
   * @param volts how many volts to set to
   */
  public void setVoltage(double volts) {
    if (m_emergencyMode == true) {
      armSim.setInputVoltage(0);
    } else {
      armSim.setInputVoltage(volts);
    }
  }

  public void update() {
    armSim.update(0.20);
    if (m_emergencyMode == false) {
      double pidOutput = pidController.calculate(getPosition());

      armSim.setInputVoltage(pidOutput);
    }
  }

  @Override
  public void setEmergencyMode(boolean emergency) {
    m_emergencyMode = emergency;
    setVoltage(0);
  }
}
