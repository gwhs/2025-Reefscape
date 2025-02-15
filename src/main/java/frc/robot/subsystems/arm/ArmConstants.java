package frc.robot.subsystems.arm;

public class ArmConstants {
  public static final int ARM_MOTOR_ID = 15;
  public static final double MAX_VELOCITY = 1.2; // rotation per second
  public static final double MAX_ACCELERATION = 6; // rotation per second per second
  public static final double ARM_GEAR_RATIO = 64.0; //12/68 * 20/84 * 16/48

  // Position consntants: in degrees
  public static final double L1_PREP_POSITION = 330;
  public static final double L2_PREP_POSITION = 330;
  public static final double L3_PREP_POSITION = 227;
  public static final double L4_PREP_POSITION = 225;

  public static final double L1_SCORE_POSITION = 150;
  public static final double L2_SCORE_POSITION = 150;
  public static final double L3_SCORE_POSITION = 150;
  public static final double L4_SCORE_POSITION = 180;
  public static final double ARM_INTAKE_ANGLE = 90.0;
  public static final double ARM_STOW_ANGLE = 90.0;
}
