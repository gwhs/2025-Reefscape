package frc.robot.subsystems.arm;

public class ArmConstants {
  public static final int ARM_MOTOR_ID = 15;

  public static final int ARM_ENCODER_ID = 20;
  public static final double MAX_VELOCITY = 1.2; // rotation per second
  public static final double MAX_ACCELERATION = 2; // rotation per second per second
  public static final double ARM_GEAR_RATIO = 68.0 / 12 * 84 / 20 * 48 / 18; // 64

  // Position constants: in degrees
  public static final double L1_PREP_POSITION = 133;
  public static final double L2_PREP_POSITION = 135;
  public static final double L3_PREP_POSITION = 124.565;
  public static final double L4_PREP_POSITION = 118;

  public static final double ARM_UPPER_BOUND = 180;
  public static final double ARM_LOWER_BOUND = -170;

  public static final double ARM_INTAKE_ANGLE = -73.477;
  public static final double ARM_INTAKE_ANGLE_BACKUP = -65.479;
  public static final double ARM_STOW_ANGLE = -90.0;

  public static final double PRE_DEALGAE_ANGLE = 155;
  public static final double DEALGAE_ANGLE = 170;

  public static final double MAGNET_OFFSET_DEGREES = -229.57;

  public static final double PREP_CLIMB_ANGLE = -170;
  public static final double CLIMB_ANGLE = -170;
}
