// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class ElevatorConstants {
  public static final int LEFT_ELEVATOR_MOTOR_ID = 21;
  public static final int RIGHT_ELEVATOR_MOTOR_ID = 14;

  public static final int GEAR_RATIO = 12;
  public static final double SPROCKET_DIAMETER = Units.inchesToMeters(1.7567);

  public static final double TOP_METER = ElevatorSubsystem.rotationsToMeters(60);

  public static final double INTAKE_METER = ElevatorSubsystem.rotationsToMeters(10);
  public static final double STOW_METER = ElevatorSubsystem.rotationsToMeters(15);
}
