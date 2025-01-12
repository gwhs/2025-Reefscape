package frc.robot.subsystems.AprilTagCam;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class AprilTagCamConstants {
  public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
  public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
  public static final Transform3d BackLeftCamLocation =
      new Transform3d(
          -0.35,
          -0.333,
          0.2921,
          new Rotation3d(
              Units.degreesToRadians(0),
              Units.degreesToRadians(-45),
              Units.degreesToRadians(-135)));

  public static final double zTolerence = 2.00;
  public static final double xyTolerance = 2.00;
  public static final double maxXValue = 690.87; 
  public static final double maxYValue = 317.00; 
  public static final double aprilTagMaxDistance = 100; 
  public static final double maxVelocity = 100;
  public static final double MAX_ROTATION = Math.PI;
  
}
