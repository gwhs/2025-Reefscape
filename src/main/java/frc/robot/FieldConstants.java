package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class FieldConstants {

  public static Pose2d[] blueReefSetpoints = {
    new Pose2d(3.285, 4.191, Rotation2d.kZero),
    new Pose2d(3.285, 3.861, Rotation2d.kZero),
    new Pose2d(3.744, 3.065, Rotation2d.fromDegrees(60)),
    new Pose2d(4.030, 2.900, Rotation2d.fromDegrees(60)),
    new Pose2d(4.949, 2.900, Rotation2d.fromDegrees(120)),
    new Pose2d(5.235, 3.065, Rotation2d.fromDegrees(120)),
    new Pose2d(5.694, 3.861, Rotation2d.fromDegrees(180)),
    new Pose2d(5.694, 4.191, Rotation2d.fromDegrees(180)),
    new Pose2d(5.235, 4.986, Rotation2d.fromDegrees(240)),
    new Pose2d(4.949, 5.151, Rotation2d.fromDegrees(240)),
    new Pose2d(4.030, 5.151, Rotation2d.fromDegrees(300)),
    new Pose2d(3.744, 4.986, Rotation2d.fromDegrees(300))
  };

  public static List<Pose2d> blueReefSetpointList =
      new ArrayList<>(Arrays.asList(blueReefSetpoints));

  public static Pose2d[] redReefSetpoints = {
    new Pose2d(11.854, 4.191, Rotation2d.kZero),
    new Pose2d(11.854, 3.861, Rotation2d.kZero),
    new Pose2d(12.314, 3.065, Rotation2d.fromDegrees(60)),
    new Pose2d(12.599, 2.900, Rotation2d.fromDegrees(60)),
    new Pose2d(13.518, 2.900, Rotation2d.fromDegrees(120)),
    new Pose2d(13.804, 3.065, Rotation2d.fromDegrees(120)),
    new Pose2d(14.263, 3.861, Rotation2d.fromDegrees(180)),
    new Pose2d(14.263, 4.191, Rotation2d.fromDegrees(180)),
    new Pose2d(13.804, 4.986, Rotation2d.fromDegrees(240)),
    new Pose2d(13.518, 5.151, Rotation2d.fromDegrees(240)),
    new Pose2d(12.599, 5.151, Rotation2d.fromDegrees(300)),
    new Pose2d(12.314, 4.986, Rotation2d.fromDegrees(300)),
  };

  public static List<Pose2d> redReefSetpointList = new ArrayList<>(Arrays.asList(redReefSetpoints));
}
