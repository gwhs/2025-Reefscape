package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
  private ElevatorIO elevatorIO;

  public ElevatorSubsystem() {
    if (RobotBase.isSimulation()) {
      elevatorIO = new ElevatorIOReal();
    } else {
      elevatorIO = new ElevatorIOSim();
    }

    SmartDashboard.putData("runHeight 1 meter", runHeight(1));
  }

  public Command runHeight(double meters) {
    return Commands.sequence(
        this.runOnce(() -> elevatorIO.runRotation(metersToRotations(meters))),
        Commands.waitUntil(
            () -> MathUtil.isNear(elevatorIO.getPIDGoalRotation(), elevatorIO.getRotation(), 0.5)));
  }

  public double getRotation() {
    return elevatorIO.getRotation();
  }

  public static double rotationsToMeters(double rotations) {
    return rotations
        / ElevatorConstants.GEAR_RATIO
        * (ElevatorConstants.SPROCKET_DIAMETER * Math.PI)
        * 1;
  }

  public static double metersToRotations(double meters) {
    return meters
        / (ElevatorConstants.SPROCKET_DIAMETER * Math.PI)
        * ElevatorConstants.GEAR_RATIO
        / 1;
  }

  @Override
  public void periodic() {
    elevatorIO.update();
  }
}
