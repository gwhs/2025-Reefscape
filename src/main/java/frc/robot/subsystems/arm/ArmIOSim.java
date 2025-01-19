package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ArmIOSim implements ArmIO {
  private SingleJointedArmSim armSim =
      new SingleJointedArmSim(
        DCMotor.getFalcon500Foc(1), 
        16, 
        0.5, 
        1, 
        Units.degreesToRadians(0), 
        Units.degreesToRadians(300), 
        false, 
        Units.degreesToRadians(0));

  private edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints constraints =
      new edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints(ArmConstants.MAX_VELOCITY, ArmConstants.MAX_ACCELERATION);
  private ProfiledPIDController pidController =
      new ProfiledPIDController(
          .1, ArmConstants.ARM_PID_KI, ArmConstants.ARM_PID_KD, constraints);

  public double getPosition() {
    return Units.radiansToDegrees(armSim.getAngleRads());
  }

@Override
public void setAngle(double angle) {
    pidController.setGoal(angle);
}
}
