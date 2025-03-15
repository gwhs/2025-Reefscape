// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class ElevatorConstants {
  public static final int FRONT_ELEVATOR_MOTOR_ID = 21;
  public static final int BACK_ELEVATOR_MOTOR_ID = 14;
  public static final int LIMIT_SWITCH_CHANNEL = 0;

  public static final int GEAR_RATIO = 12;
  public static final double SPROCKET_DIAMETER = Units.inchesToMeters(1.7567);

  public static final double TOP_METER = .7593;

  // Position constants:
  public static final double L1_PREP_POSITION = 0;
  public static final double L2_PREP_POSITION = 0;
  public static final double L3_PREP_POSITION = 0.2;
  public static final double L4_PREP_POSITION = .7593;

  public static final double INTAKE_METER = Units.inchesToMeters(10.5);
  public static final double DEALGAE_LOW_POSITION = 0;
  public static final double DEALGAE_HIGH_POSITION = .4424;
  public static final double DEALGAE_STOW_POSITION = 0;

  public static final double STOW_METER = .1752;

  public static final double MAX_VELOCITY = 80; // rps
  public static final double MAX_ACCELERATION = 800; // rps
}
