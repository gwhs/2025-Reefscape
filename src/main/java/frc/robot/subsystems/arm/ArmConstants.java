package frc.robot.subsystems.arm;

public class ArmConstants {
  public static final int ARM_MOTOR_ID = 15;

  public static final int ARM_ENCODER_ID = 9;
  public static final double MAX_VELOCITY = 1.2; // rotation per second
  public static final double MAX_ACCELERATION = 5.0; // rotation per second per second
  public static final double ARM_GEAR_RATIO = 68.0 / 12 * 84 / 20 * 48 / 18; // 64

  // Position consntants: in degrees
  public static final double L1_PREP_POSITION = 313;
  public static final double L2_PREP_POSITION = 315;
  public static final double L3_PREP_POSITION = 300;
  public static final double L4_PREP_POSITION = 300;

  public static final double ARM_INTAKE_ANGLE = 130.0;
  public static final double ARM_STOW_ANGLE = 75.0;
}
