package frc.robot.subsystems.objectDetection;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class ObjectDetectionConstants {

  private final double minConfindence = 0.75;
  public static final Transform3d robotToCam = // placeholder values, change later
      new Transform3d(
          Units.inchesToMeters(9.6),
          Units.inchesToMeters(11.2),
          Units.inchesToMeters(8),
          new Rotation3d(
              Units.degreesToRadians(0), Units.degreesToRadians(-20), Units.degreesToRadians(-5)));

  public static final double Z_TOLERANCE = 2.00;
  public static final double XY_TOLERANCE = 2.00;
  public static final double MAX_X_VALUE = 690.87;
  public static final double MAX_Y_VALUE = 317.00;
  public static final double APRILTAG_MAX_DISTANCE = 2.4;
  public static final double MAX_VELOCITY = 4;
  public static final double MAX_ROTATION = Math.PI;
            
}


