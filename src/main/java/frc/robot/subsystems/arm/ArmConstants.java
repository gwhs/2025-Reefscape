package frc.robot.subsystems.arm;

public class ArmConstants {
  public static final int ARM_MOTOR_ID = 15;
  public static final double MAX_VELOCITY = 80; // rotation per second
  public static final double MAX_ACCELERATION = 80; // rotation per second per second
  public static final double ARM_GEAR_RATIO = 64.0;

  // Position consntants: in degrees
  public static final double VERTICAL_POSITION = 0.0;
  public static final double L3_PREP_POSITION = 200;
  public static final double L3_RELEASE_POSITION = 150;
  public static final double L4_PREP_POSITION = 255;
  public static final double L4_RELEASE_POSITION = 265;
  public static final double ARM_INTAKE_ANGLE = 90.0;
}
