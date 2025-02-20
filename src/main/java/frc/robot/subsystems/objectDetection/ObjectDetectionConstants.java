package frc.robot.subsystems.objectDetection;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class ObjectDetectionConstants {

  private final double minConfindence = 0.75;
  public static final Transform3d robotToCam =
      new Transform3d(
          Units.inchesToMeters(9.6),
          Units.inchesToMeters(11.2),
          Units.inchesToMeters(8),
          new Rotation3d(
              Units.degreesToRadians(0), Units.degreesToRadians(-20), Units.degreesToRadians(-5)));
}
