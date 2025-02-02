package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.List;

public class FieldConstants {

  public static List<Pose2d> blueReefSetpointList = EagleUtil.calculateBlueReefSetPoints();

  public static List<Pose2d> redReefSetpointList = EagleUtil.calculateRedReefSetPoints();
}
