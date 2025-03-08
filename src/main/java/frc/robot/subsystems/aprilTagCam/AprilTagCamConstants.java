package frc.robot.subsystems.aprilTagCam;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class AprilTagCamConstants {
  public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(1, 1, 8);
  public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

  public static final String FRONT_LEFT_CAMERA_COMP_NAME = "leftcam";
  public static final String FRONT_RIGHT_CAMERA_COMP_NAME = "rightcam";
  public static final String FRONT_LEFT_CAMERA_DEV_NAME = "cam3";
  public static final String FRONT_RIGHT_CAMERA_DEV_NAME = "cam4";

  public static final Transform3d FRONT_RIGHT_CAMERA_LOCATION_DEV =
      new Transform3d(
          Units.inchesToMeters(9.524),
          Units.inchesToMeters(-11.185),
          Units.inchesToMeters(8.250),
          new Rotation3d(
              Units.degreesToRadians(0), Units.degreesToRadians(-10), Units.degreesToRadians(5)));

  public static final Transform3d FRONT_LEFT_CAMERA_LOCATION_DEV =
      new Transform3d(
          Units.inchesToMeters(9.524),
          Units.inchesToMeters(11.185),
          Units.inchesToMeters(8.250),
          new Rotation3d(
              Units.degreesToRadians(0), Units.degreesToRadians(-10), Units.degreesToRadians(-5)));

  public static final Transform3d FRONT_RIGHT_CAMERA_LOCATION_COMP =
      new Transform3d(
          Units.inchesToMeters(6.200),
          Units.inchesToMeters(-11.468),
          Units.inchesToMeters(7.561),
          new Rotation3d(
              Units.degreesToRadians(0), Units.degreesToRadians(-20), Units.degreesToRadians(5)));

  public static final Transform3d FRONT_LEFT_CAMERA_LOCATION_COMP =
      new Transform3d(
          Units.inchesToMeters(6.200),
          Units.inchesToMeters(11.968),
          Units.inchesToMeters(7.561),
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
