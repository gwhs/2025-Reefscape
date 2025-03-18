package frc.robot.subsystems.arm;

public class ArmConstants {
  public static final int ARM_MOTOR_ID = 15;

  public static final int ARM_ENCODER_ID = 20;
  public static final double MAX_VELOCITY = 1.2; // rotation per second
  public static final double MAX_ACCELERATION = 2; // rotation per second per second
  public static final double ARM_GEAR_RATIO = 68.0 / 12 * 84 / 20 * 48 / 18; // 64

  // Position constants: in degrees
  public static final double L1_PREP_POSITION = 313;
  public static final double L2_PREP_POSITION = 315;
  public static final double L3_PREP_POSITION = 304.565;
  public static final double L4_PREP_POSITION = 304.565;

  public static final double ARM_UPPER_BOUND = 360;
  public static final double ARM_LOWER_BOUND = 10;

  public static final double ARM_INTAKE_ANGLE = 106.523;
  public static final double ARM_INTAKE_ANGLE_BACKUP = 0;
  public static final double ARM_STOW_ANGLE = 90.0;

  public static final double PRE_DEALGAE_ANGLE = 335;
  public static final double DEALGAE_ANGLE = 350;

  public static final double MAGNET_OFFSET_DEGREES = -49.57;

  public static final double PREP_CLIMB_ANGLE = 10;
  public static final double CLIMB_ANGLE = 10;
}
