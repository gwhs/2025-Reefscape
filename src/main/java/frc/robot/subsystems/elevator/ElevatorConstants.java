// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class ElevatorConstants {
  public static final int FRONT_ELEVATOR_MOTOR_ID = 21;
  public static final int BACK_ELEVATOR_MOTOR_ID = 14;

  public static final int GEAR_RATIO = 16;
  public static final double SPROCKET_DIAMETER = Units.inchesToMeters(1.7567);

  public static final double TOP_METER = ElevatorSubsystem.rotationsToMeters(60 / 12 * 16);

  // Position constants:
  public static final double L1_PREP_POSITION = ElevatorSubsystem.rotationsToMeters(0 / 12 * 16);
  public static final double L2_PREP_POSITION = ElevatorSubsystem.rotationsToMeters(0 / 12 * 16);
  public static final double L3_PREP_POSITION = ElevatorSubsystem.rotationsToMeters(5 / 12 * 16);
  public static final double L4_PREP_POSITION = ElevatorSubsystem.rotationsToMeters(60 / 12 * 16);

  public static final double L1_SCORE_POSITION = ElevatorSubsystem.rotationsToMeters(0 / 12 * 16);
  public static final double L2_SCORE_POSITION = ElevatorSubsystem.rotationsToMeters(0 / 12 * 16);

  public static final double L3_SCORE_POSITION = ElevatorSubsystem.rotationsToMeters(5 / 12 * 16);
  public static final double L4_SCORE_POSITION = ElevatorSubsystem.rotationsToMeters(59 / 12 * 16);

  public static final double INTAKE_METER = ElevatorSubsystem.rotationsToMeters(0);
  public static final double STOW_METER = ElevatorSubsystem.rotationsToMeters(15 / 12 * 16);

  public static final double MAX_VELOCITY = 90; // rps
  public static final double MAX_ACCELERATION = 400; // rps
}
