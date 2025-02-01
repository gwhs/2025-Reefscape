package frc.robot.subsystems.arm;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
  private ArmIO armIO;

  public ArmSubsystem() {
    if (RobotBase.isSimulation()) {
      armIO = new ArmIOSim();
    } else {
      armIO = new ArmIOReal();
    }

    SmartDashboard.putData("Arm Command/turn to 60 degrees", setAngle(60));
    SmartDashboard.putData("Arm Command/turn to 120 degrees", setAngle(120));
    SmartDashboard.putData("Arm Command/turn to 270 degrees", setAngle(270));
    SmartDashboard.putData("Arm Command/turn to 0 degrees", setAngle(0));
  }

  /**
   * drives the arm until it reaches the given provided angle
   *
   * @param angle Angle to drive the arm to in degrees TODO Morgan add sign information/0 position
   *     information to param annotation
   */
  public Command setAngle(double angle) {
    double clampedAngle = MathUtil.clamp(angle, 0, 360);
    return this.runOnce(
            () -> {
              armIO.setAngle(clampedAngle);
            })
        .andThen(Commands.waitUntil(() -> MathUtil.isNear(clampedAngle, armIO.getPosition(), 0.1)));
  }

  @Override
  public void periodic() {
    armIO.update();
    DogLog.log("Arm/arm angle", armIO.getPosition());
  }

  public double getAngle() {
    return armIO.getPosition();
  }
}
