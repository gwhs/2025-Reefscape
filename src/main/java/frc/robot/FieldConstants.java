package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class FieldConstants {

  public static Pose2d[] blueReefSetpoints = {
    new Pose2d(2.904, 4.191, Rotation2d.kZero),
    new Pose2d(2.904, 3.861, Rotation2d.kZero),
    new Pose2d(3.554, 2.735, Rotation2d.fromDegrees(60)),
    new Pose2d(3.840, 2.570, Rotation2d.fromDegrees(60)),
    new Pose2d(5.139, 2.570, Rotation2d.fromDegrees(120)),
    new Pose2d(5.425, 2.735, Rotation2d.fromDegrees(120)),
    new Pose2d(6.075, 3.861, Rotation2d.fromDegrees(180)),
    new Pose2d(6.075, 4.191, Rotation2d.fromDegrees(180)),
    new Pose2d(5.425, 5.316, Rotation2d.fromDegrees(240)),
    new Pose2d(5.139, 5.481, Rotation2d.fromDegrees(240)),
    new Pose2d(3.840, 5.481, Rotation2d.fromDegrees(300)),
    new Pose2d(3.554, 5.316, Rotation2d.fromDegrees(300))
  };

  public static List<Pose2d> blueReefSetpointList =
      new ArrayList<>(Arrays.asList(blueReefSetpoints));

  public static Pose2d[] redReefSetpoints = {
    new Pose2d(11.473, 4.191, Rotation2d.kZero),
    new Pose2d(11.473, 3.861, Rotation2d.kZero),
    new Pose2d(12.123, 2.735, Rotation2d.fromDegrees(60)),
    new Pose2d(12.409, 2.570, Rotation2d.fromDegrees(60)),
    new Pose2d(13.709, 2.570, Rotation2d.fromDegrees(120)),
    new Pose2d(13.994, 2.735, Rotation2d.fromDegrees(120)),
    new Pose2d(14.644, 3.861, Rotation2d.fromDegrees(180)),
    new Pose2d(14.644, 4.191, Rotation2d.fromDegrees(180)),
    new Pose2d(13.994, 5.316, Rotation2d.fromDegrees(240)),
    new Pose2d(13.709, 5.481, Rotation2d.fromDegrees(240)),
    new Pose2d(12.409, 5.481, Rotation2d.fromDegrees(300)),
    new Pose2d(12.123, 5.316, Rotation2d.fromDegrees(300)),
  };

  public static List<Pose2d> redReefSetpointList = new ArrayList<>(Arrays.asList(redReefSetpoints));
}
