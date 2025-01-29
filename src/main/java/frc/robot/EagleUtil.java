package frc.robot;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class EagleUtil {
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
  private static double ROBOT_AWAY_FROM_REEF = Units.inchesToMeters(15);
  private static double X = -REEF_LENGTH - ROBOT_AWAY_FROM_REEF;
  private static double Y = REEF_TO_REEF_DISTANCE / 2;
  private static double Y_OFFSET = -0.1; // to be changed if/when needed
  private static double[][] blueReefOffsets = {{0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}};
  private static double[][] redReefOffsets = {{0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}};

  // blue poses
  private static Pose2d[] bluePoses = new Pose2d[12];

  // red poses
  private static Pose2d[] redPoses = new Pose2d[12];

  public static void calculateBlueReefSetPoints() {
    
    bluePoses[0] = new Pose2d(X, Y + Y_OFFSET, Rotation2d.kZero);
    bluePoses[1] = new Pose2d(X, -Y + Y_OFFSET, Rotation2d.kZero);

    Rotation2d sixty = Rotation2d.fromDegrees(60);

    for (int i = 2; i < bluePoses.length; i++) {
      bluePoses[i] = bluePoses[i - 2].rotateBy(sixty);
    }

    for (int i = 0; i < bluePoses.length; i++) {
      bluePoses[i] = bluePoses[i].relativeTo(BLUE_REEF_INVERT);
      bluePoses[i] = new Pose2d(bluePoses[i].getX() + blueReefOffsets[i][0], bluePoses[i].getY() + blueReefOffsets[i][1], bluePoses[i].getRotation());
    }

    

    DogLog.log("Caculation/Blue Reef", BLUE_REEF);
    DogLog.log("Caculation/Blue Set Points", bluePoses);
  }

  public static void calculateRedReefSetPoints() {
    redPoses[0] = new Pose2d(X, Y + Y_OFFSET, Rotation2d.kZero);
    redPoses[1] = new Pose2d(X, -Y + Y_OFFSET, Rotation2d.kZero);
    
    Rotation2d sixty = Rotation2d.fromDegrees(60);

    for (int i = 2; i < redPoses.length; i++) {
      redPoses[i] = redPoses[i - 2].rotateBy(sixty);
    }

    for (int i = 0; i < redPoses.length; i++) {
      redPoses[i] = redPoses[i].relativeTo(RED_REEF_INVERT);
      redPoses[i] = new Pose2d(bluePoses[i].getX() + redReefOffsets[i][0], bluePoses[i].getY() + redReefOffsets[i][1], bluePoses[i].getRotation());
    }
    DogLog.log("Caculation/Red Reef", RED_REEF);
    DogLog.log("Caculation/Red Set Points", redPoses);
  }
}
