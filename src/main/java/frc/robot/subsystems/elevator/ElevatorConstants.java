package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.util.Units;

public class ElevatorConstants {

  public static CANBus ELEVATOR_MOTOR_CAN_BUS = new CANBus("rio");
  public static int FRONT_ELEVATOR_MOTOR_ID = 14;
  public static int BACK_ELEVATOR_MOTOR_ID = 21;
  public static final double MAX_VELOCITY = 80;
  public static final double MAX_ACCELERATION = 330;
  public static final int GEAR_RATIO = 12;
  public static final double SPROCKET_DIAMETER = Units.inchesToMeters(1.7567);
  public static final double TOP_METER = .75;
}
