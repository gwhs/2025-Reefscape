package frc.robot.subsystems.groundIntake;

public class GroundIntakeConstants {

  public static final int SPIN_MOTOR_ID = 31;
  public static final int PIVOT_MOTOR_ID = 32;
  public static final int PIVOT_ENCODER_ID = 33;

  public static final double PIVOT_GEAR_RATIO = 48.222;
  public static final double SPIN_GEAR_RATIO = 2.0;

  public static final double ENCODER_OFFSET = 295.0;

  public static final double GROUND_INTAKE_LOWER_BOUND = -130.0;
  public static final double GROUND_INTAKE_UPPER_BOUND = 0.0;

  public static final double MAX_VELOCITY = 1.2;
  public static final double MAX_ACCELERATION = 2;

  public static final double INTAKE_CORAL_ANGLE = -95; // TODO
  public static final double CORAL_STOW_ANGLE = 0; // TODO

  public static final double INTAKE_ALGAE_ANGLE = -45; // TODO
  public static final double ALGAE_STOW_ANGLE = -10; // TODO

  public static final double SCORE_ALGAE_ANGLE = -10; // TODO
  public static final double SCORE_CORAL_ANGLE = -5; // TODO

  public static final double CLIMB_ANGLE = -15; // TODO

  public static final double INTAKE_CORAL_AMP = 20;
  public static final double INTAKE_CORAL_DUTYCYCLE = 1;
  public static final double HOLD_CORAL_AMP = 10;
  public static final double HOLD_CORAL_DUTYCYCLE = 0.1;
  public static final double SCORE_CORAL_AMP = -20;
  public static final double SCORE_CORAL_DUTYCYCLE = -0.9;

  public static final double MAGNET_OFFSET_ROTATIONS = 0.36;
}
