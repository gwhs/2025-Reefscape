// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class ElevatorConstants {
  public static final double CLIMBER_RATIO = 36;

  public static final int LEFT_ELEVATOR_MOTOR_ID = 21;
  public static final boolean LEFT_ELEVATOR_MOTOR_INVERTED = false;
  public static final boolean RIGHT_ELEVATOR_MOTOR_INVERTED = true;
  public static final int RIGHT_ELEVATOR_MOTOR_ID = 14;

  public static final int BOT_RIGHT_LIMIT_ID = 2;
  public static final int BOT_LEFT_LIMIT_ID = 3;
  public static final int TOP_RIGHT_LIMIT_ID = 4;
  public static final int TOP_LEFT_LIMIT_ID = 5;

  public static final double ELEVATOR_PID_KP = 0.17;
  public static final double ELEVATOR_PID_KI = 0;
  public static final double ELEVATOR_PID_KD = 0;

  public static final double MAX_ACCELERATION = 150.0;
  public static final double MAX_VELOCITY = 300.0;

  public static final double LEFT_UP_POSITION = -193.94;
  public static final double LEFT_DOWN_POSITION = -12.3;
  public static final double RIGHT_UP_POSITION = 193.4;
  public static final double RIGHT_DOWN_POSITION = 12.3;

  public static final int ForwardLimitRemoteSensorID = 0;
  public static final int ReverseLimitRemoteSensorID = 0;

  public static final int GEAR_RATIO = 16;
  public static final double SPROCKET_DIAMETER = Units.inchesToMeters(1.757);

  public static final double TOP_METER = 1.8;
}
