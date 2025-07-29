package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.List;

public class FieldConstants {

  public static List<Pose2d> blueReefSetpointList = EagleUtil.calculateBlueReefSetPoints();

  public static List<Pose2d> redReefSetpointList = EagleUtil.calculateRedReefSetPoints();

  public static List<Pose2d> blueAlgaeSetpointList = EagleUtil.calculateBlueAlgaeSetPoints();
  public static List<Pose2d> redAlgaeSetpointList = EagleUtil.calculateRedAlgaeSetPoints();

  public static final double RED_LEFT_STATION_ANGLE = 126;
  public static final double RED_RIGHT_STATION_ANGLE = -126;
  public static final double BLUE_LEFT_STATION_ANGLE = 54;
  public static final double BLUE_RIGHT_STATION_ANGLE = -54;

  public static final double BLUE_CAGE_ANGLE = 90;
  public static final double RED_CAGE_ANGLE = -90;

  // Unit is meters
  public static final double HALF_WIDTH_FIELD = 4.0359;
}
