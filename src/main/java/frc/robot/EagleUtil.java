package frc.robot;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;

public class EagleUtil {
  public static ArrayList<Pose2d> m_redPoses;
  public static ArrayList<Pose2d> m_bluePoses;

  private static double BLUE_REEF_X = Units.inchesToMeters(144 + (93.5 - 14 * 2) / 2);
  private static double BLUE_REEF_Y = Units.inchesToMeters(158.50);
  private static Pose2d BLUE_REEF = new Pose2d(BLUE_REEF_X, BLUE_REEF_Y, Rotation2d.kZero);
  private static Pose2d BLUE_REEF_INVERT = new Pose2d(-BLUE_REEF_X, -BLUE_REEF_Y, Rotation2d.kZero);
  private static double RED_REEF_X = Units.inchesToMeters(546.875 - (93.5 - 14 * 2) / 2);
  private static double RED_REEF_Y = Units.inchesToMeters(158.50);
  private static Pose2d RED_REEF = new Pose2d(RED_REEF_X, RED_REEF_Y, Rotation2d.kZero);
  private static Pose2d RED_REEF_INVERT = new Pose2d(-RED_REEF_X, -RED_REEF_Y, Rotation2d.kZero);

  private static double REEF_LENGTH = Units.inchesToMeters(35);
  private static double REEF_TO_REEF_DISTANCE = 0.33;
  private static double ROBOT_AWAY_FROM_REEF = Units.inchesToMeters(17);

  private static double X = -REEF_LENGTH - ROBOT_AWAY_FROM_REEF;
  private static double Y = REEF_TO_REEF_DISTANCE / 2;
  private static double Y_OFFSET = Units.inchesToMeters(0.4);

  private static Pose2d[] bluePoses = new Pose2d[12];
  private static Pose2d[] redPoses = new Pose2d[12];

  private static Pose2d cachedPose = null;
  private static Alliance red = DriverStation.Alliance.Red;

  public static ArrayList<Pose2d> calculateBlueReefSetPoints() {
    if (m_bluePoses != null) {
      return m_bluePoses;
    }

    double[][] blueReefOffsets = {
      // in meters
      {0, 0}, // reef A
      {0, 0}, // reef B
      {0, 0}, // reef C
      {0, 0}, // reef D
      {0, 0}, // reef E
      {0, 0}, // reef F
      {0, 0}, // reef G
      {0, 0}, // reef H
      {0, 0}, // reef I
      {0, 0}, // reef J
      {0, 0}, // reef K
      {0, 0} // reef L
    };

    bluePoses[0] = new Pose2d(X, Y + Y_OFFSET, Rotation2d.kZero);
    bluePoses[1] = new Pose2d(X, -Y - Y_OFFSET, Rotation2d.kZero);

    Rotation2d sixty = Rotation2d.fromDegrees(60);

    for (int i = 2; i < bluePoses.length; i++) {
      bluePoses[i] = bluePoses[i - 2].rotateBy(sixty);
    }

    for (int i = 0; i < bluePoses.length; i++) {
      bluePoses[i] = bluePoses[i].relativeTo(BLUE_REEF_INVERT);
      bluePoses[i] =
          new Pose2d(
              bluePoses[i].getX() + blueReefOffsets[i][0],
              bluePoses[i].getY() + blueReefOffsets[i][1],
              bluePoses[i].getRotation());
    }

    DogLog.log("Caculation/Blue Reef", BLUE_REEF);
    DogLog.log("Caculation/Blue Set Points", bluePoses);
    m_bluePoses = new ArrayList<Pose2d>(Arrays.asList(bluePoses));
    return m_bluePoses;
  }

  public static ArrayList<Pose2d> calculateRedReefSetPoints() {
    if (m_redPoses != null) {
      return m_redPoses;
    }

    redPoses[0] = new Pose2d(X, Y + Y_OFFSET, Rotation2d.kZero);
    redPoses[1] = new Pose2d(X, -Y + Y_OFFSET, Rotation2d.kZero);

    double[][] redReefOffsets = {
      // also in meters
      {0, 0}, // reef G
      {0, 0}, // reef H
      {0, 0}, // reef I
      {0, 0}, // reef J
      {0, 0}, // reef K
      {0, 0}, // reef L
      {0, 0}, // reef A
      {0, 0}, // reef B
      {0, 0}, // reef C
      {0, 0}, // reef D
      {0, 0}, // reef E
      {0, 0} // reef F
    };

    Rotation2d sixty = Rotation2d.fromDegrees(60);

    for (int i = 2; i < redPoses.length; i++) {
      redPoses[i] = redPoses[i - 2].rotateBy(sixty);
    }

    for (int i = 0; i < redPoses.length; i++) {
      redPoses[i] = redPoses[i].relativeTo(RED_REEF_INVERT);
      redPoses[i] =
          new Pose2d(
              redPoses[i].getX() + redReefOffsets[i][0],
              redPoses[i].getY() + redReefOffsets[i][1],
              redPoses[i].getRotation());
    }
    DogLog.log("Caculation/Red Reef", RED_REEF);
    DogLog.log("Caculation/Red Set Points", redPoses);

    m_redPoses = new ArrayList<Pose2d>(Arrays.asList(redPoses));

    return m_redPoses;
  }

  private static Pose2d getNearestReefPoint(Pose2d pose) {
    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
      return pose.nearest(FieldConstants.blueReefSetpointList);
    } else {
      return pose.nearest(FieldConstants.redReefSetpointList);
    }
  }

  public static Pose2d getCachedReefPose(Pose2d pose) {
    if (cachedPose == null) {
      cachedPose = getNearestReefPoint(pose);
    }
    return cachedPose;
  }

  public static void clearCachedPose() {
    cachedPose = null;
  }

  public static double getDistanceBetween(Pose2d poseA, Pose2d poseB) {
    return poseA.getTranslation().getDistance(poseB.getTranslation());
  }

  public static class PoseComparator implements Comparator<Pose2d> {
    public Pose2d robotPose;

    public PoseComparator(Pose2d robotPose) {
      this.robotPose = robotPose;
    }

    public int compare(Pose2d poseA, Pose2d poseB) {
      double distA = poseA.getTranslation().getDistance(robotPose.getTranslation());
      double distB = poseB.getTranslation().getDistance(robotPose.getTranslation());
      if (distA > distB) {
        return 1;
      } else if (distA == distB) {
        return 0;
      }
      return -1;
    }
  }

  public static Pose2d closestReefSetPoint(Pose2d pose, int n) {
    n = MathUtil.clamp(n, 0, 23);
    if (isRedAlliance()) {
      ArrayList<Pose2d> red = calculateRedReefSetPoints();
      red.sort(new PoseComparator(pose));
      return red.get(n);
    }

    ArrayList<Pose2d> blue = calculateBlueReefSetPoints();
    blue.sort(new PoseComparator(pose));
    return blue.get(n);
  }

  public static boolean isRedAlliance() {
    return DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == red;
  }

  public static Command triggerAlert(Alert alert) {
    return Commands.runOnce(() -> alert.set(true));
  }

  public static Command detriggerAlert(Alert alert) {
    return Commands.runOnce(() -> alert.set(false));
  }

  public static Pose2d blueCoralStationProcessorSide = new Pose2d(1.0, 0.27, Rotation2d.kZero);
  public static Pose2d blueCoralStationNonProcessorSide = new Pose2d(1.0, 7.313, Rotation2d.kZero);
  public static Pose2d redCoralStationProcessorSide = new Pose2d(16.5, 0.27, Rotation2d.kZero);
  public static Pose2d redCoralStationNonProcessorSide = new Pose2d(16.5, 7.415, Rotation2d.kZero);

  public static Pose2d getProcessorForAlliance() {
    if (isRedAlliance()) {
      return redCoralStationProcessorSide;
    }
    return blueCoralStationProcessorSide;
  }

  public static Pose2d getNonProcessorForAlliance() {
    if (isRedAlliance()) {
      return redCoralStationNonProcessorSide;
    }
    return blueCoralStationNonProcessorSide;
  }

  public static Pose2d getClosetStationGen(Pose2d pose) {
    if (getDistanceBetween(pose, getNonProcessorForAlliance())
        < getDistanceBetween(pose, getProcessorForAlliance())) {
      return getNonProcessorForAlliance();
    }
    return getProcessorForAlliance();
  }
}
