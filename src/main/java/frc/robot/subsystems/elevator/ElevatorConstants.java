package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.util.Units;

public class ElevatorConstants {

  public static CANBus ElevatorMotorCanBus = new CANBus("rio");
  public static int FrontElevatorMotorID = 14;
  public static int BackElevatorMotorID = 21;
  public static final double MAX_VELOCITY = 80;
  public static final double MAX_ACCELERATION = 330;
  public static final int GEAR_RATIO = 12;
  public static final double SPROCKET_DIAMETER = Units.inchesToMeters(1.7567);
  public static final double TOP_METER = .75;
}
