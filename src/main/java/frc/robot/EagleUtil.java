package frc.robot;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class EagleUtil {
  private static double BLUE_REEF_X = Units.inchesToMeters(144 + (93.5 - 14 * 2) / 2);
  private static double BLUE_REEF_Y = Units.inchesToMeters(158.50);
  private static Pose2d BLUE_REEF = new Pose2d(blue_reef_x, blue_reef_y, Rotation2d.kZero);
  private static Pose2d BLUE_REEF_INVERT = new Pose2d(-blue_reef_x, -blue_reef_y, Rotation2d.kZero);

  private static double RED_REEF_X = Units.inchesToMeters(546.875 - (93.5 - 14 * 2) / 2);
  private static double RED_REEF_Y = Units.inchesToMeters(158.50);
  private static Pose2d RED_REEF = new Pose2d(red_reef_x, red_reef_y, Rotation2d.kZero);
  private static Pose2d RED_REEF_INVERT = new Pose2d(-red_reef_x, -red_reef_y, Rotation2d.kZero);

  private static double REEF_LENGTH = (2.37 / 2) * Math.tanh(Units.degreesToRadians(30));

  private static double REEF_TO_REEF_DISTANCE = 0.33;

  private static double ROBOT_AWAY_FROM_REEF = Units.inchesToMeters(50) / 2;

  private static double X = -reef_length - robot_away_from_reef;
  private static double Y = reef_to_reef_distance / 2;

  // blue poses
  private static Pose2d[] bluePoses = new Pose2d[12];

  // red poses
  private static Pose2d[] redPoses = new Pose2d[12];

  public static void printBlueReefSetPoints() {

    for (int i = 0; i < bluePoses.length; i++) {
      System.out.print(bluePoses[i] + ", ");
    }
    bluePoses[0] = new Pose2d(x, y, Rotation2d.kZero);
    bluePoses[1] = new Pose2d(x, -y, Rotation2d.kZero);

    Rotation2d sixty = Rotation2d.fromDegrees(60);

    for (int i = 2; i < bluePoses.length; i++) {
      bluePoses[i] = bluePoses[i - 2].rotateBy(sixty);
    }

    for (int i = 0; i < bluePoses.length; i++) {
      bluePoses[i] = bluePoses[i].relativeTo(blue_reef_invert);
    }

    DogLog.log("Blue Reef", blue_reef);
    DogLog.log("Blue Set Points", bluePoses);
  }

  public static void printRedReefSetPoints() {
    for (int i = 0; i < redPoses.length; i++) {
      System.out.print(redPoses[i] + ", ");
    }
    redPoses[0] = new Pose2d(x, y, Rotation2d.kZero);
    redPoses[1] = new Pose2d(x, -y, Rotation2d.kZero);

    Rotation2d sixty = Rotation2d.fromDegrees(60);

    for (int i = 2; i < redPoses.length; i++) {
      redPoses[i] = redPoses[i - 2].rotateBy(sixty);
    }

    for (int i = 0; i < redPoses.length; i++) {
      redPoses[i] = redPoses[i].relativeTo(red_reef_invert);
    }
    DogLog.log("Red Reef", red_reef);
    DogLog.log("Red Set Points", redPoses);
  }
}
