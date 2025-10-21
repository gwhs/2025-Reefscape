package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ElevatorSubsystem {
  private ElevatorIO elevatorIO;

  public ElevatorSubsystem() {
    if (RobotBase.isSimulation()) {
      elevatorIO = new ElevatorIOReal();
    } else {
      elevatorIO = new ElevatorIOSim();
    }
  }

  public Command setRotation(double rotation) {
    return Commands.run(() -> elevatorIO.setRotation(rotation));
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
}
