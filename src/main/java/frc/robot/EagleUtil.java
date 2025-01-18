package frc.robot;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class EagleUtil {
  double blue_reef_x = Units.inchesToMeters(144 + (93.5 - 14 * 2) / 2);
  double blue_reef_y = Units.inchesToMeters(158.50);
  Pose2d blue_reef = new Pose2d(blue_reef_x, blue_reef_y, Rotation2d.kZero);
  Pose2d blue_reef_invert = new Pose2d(blue_reef_x, blue_reef_y, Rotation2d.kZero);

  double red_reef_x = Units.inchesToMeters(690.875 - (93.5 - 14 * 2) / 2);
  double red_reef_y = Units.inchesToMeters(158.50);
  Pose2d red_reef = new Pose2d(red_reef_x, red_reef_y, Rotation2d.kZero);
  Pose2d red_reef_invert = new Pose2d(red_reef_x, red_reef_y, Rotation2d.kZero);
  

  double reef_length = (2.37 / 2) * Math.tanh(Units.degreesToRadians(30));

  double reef_to_reef_distance = 0.33;

  double robot_away_from_reef = Units.inchesToMeters(50) / 2;

  double x = -reef_length - robot_away_from_reef;
  double y = reef_to_reef_distance / 2;

  Pose2d[] poses = new Pose2d[12];

  {
    poses[0] = new Pose2d(x, y, Rotation2d.kZero);
    poses[1] = new Pose2d(x, -y, Rotation2d.kZero);

    Rotation2d sixty = Rotation2d.fromDegrees(60);

    for (int i = 2; i < poses.length; i++) {
      poses[i] = poses[i - 2].rotateBy(sixty);
    }

    for (int i = 0; i < poses.length; i++) {
      poses[i] = poses[i].relativeTo(blue_reef_invert);
    }
    DogLog.log("Reef", blue_reef);
    DogLog.log("Set Point", poses);
  }
  //red poses
  Pose2d[] redPoses = new Pose2d[12];

  {
    redPoses[0] = new Pose2d(x, y, Rotation2d.kZero);
    redPoses[1] = new Pose2d(x, -y, Rotation2d.kZero);

    Rotation2d sixty = Rotation2d.fromDegrees(60);

    for (int i = 2; i < redPoses.length; i++) {
      redPoses[i] = redPoses[i - 2].rotateBy(sixty);
    }

    for (int i = 0; i < poses.length; i++) {
      redPoses[i] = redPoses[i].relativeTo(red_reef_invert);
    }
    DogLog.log("Red Reef", red_reef);
    DogLog.log("Red Set Points", redPoses);
  }

  public void printCoordinates() {
    for (int i = 0; i < poses.length; i++) {
        System.out.println("Pose " + i + ": (" + poses[i].getX() + ", " + poses[i].getY() + ", " + poses[i].getRotation().getDegrees() + " degrees)");
    }

  
  

  }
}
