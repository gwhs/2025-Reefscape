// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsytems.AprilTagCam;

import dev.doglog.DogLog;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/** Add your docs here. */
public class AprilTagCam {
  AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

  private final PhotonCamera cam;
  private final Consumer<AprilTagHelp> addVisionMeasurement;
  private final PhotonPoseEstimator photonEstimator;
  private final Transform3d robotToCam;
  private final Supplier<Pose2d> currRobotPose;
  private AprilTagHelp helper;

  private final String ntKey;

  Optional<EstimatedRobotPose> optionalEstimPose;

  //
  public AprilTagCam(
      String str,
      Transform3d robotToCam,
      Consumer<AprilTagHelp> addVisionMeasurement,
      Supplier<Pose2d> currRobotPose) {
    cam = new PhotonCamera(str);
    this.addVisionMeasurement = addVisionMeasurement;
    this.robotToCam = robotToCam;
    this.currRobotPose = currRobotPose;

    photonEstimator =
        new PhotonPoseEstimator(
            aprilTagFieldLayout, PhotonPoseEstimator.PoseStrategy.AVERAGE_BEST_TARGETS, robotToCam);

    ntKey = "/Vision/" + str + "/";
  }

  public void updatePoseEstim() {

    Pose2d robotPose = currRobotPose.get();
    Pose3d robotPose3d = new Pose3d(robotPose);
    Pose3d cameraPose3d = robotPose3d.plus(robotToCam);
    DogLog.log(ntKey + "Camera Pose/", cameraPose3d);

    // write an if statement that allows to find if the the list is empty or not
    // getting the unread results target pose
    // we need to get the robot pose from the target pose
    // using the update method, in photonPoseEstimator, get an estimated robot pose
    // using this we can get pose3D and turn it into pose2d
    // we need to give the info of where the robot is to the drive train so it knows where to move

    List<PhotonPipelineResult> results = cam.getAllUnreadResults();
    if (results.isEmpty()) {
      return;
    }
    for (PhotonPipelineResult targetPose : results) {

      System.out.println(targetPose);
      System.out.println(targetPose);

      optionalEstimPose = photonEstimator.update(targetPose);

      if (optionalEstimPose.isEmpty()) {
        return;
      }

      Pose3d estimPose3d;
      estimPose3d = optionalEstimPose.get().estimatedPose;

      Pose2d pos = estimPose3d.toPose2d(); // yay :0 im so happy
      double timestamp = targetPose.getTimestampSeconds();
      Matrix<N3, N1> sd = findSD(optionalEstimPose, null);

      helper = new AprilTagHelp(pos, timestamp, sd);

      DogLog.log(ntKey + "Accepted Pose/", pos);
      DogLog.log(ntKey + "Accepted Time Stamp/", timestamp);
      DogLog.log(ntKey + "Accepted Stdev/", sd);

      addVisionMeasurement.accept(helper);
      System.out.println(
          "X: " + estimPose3d.getX() + ", Y:" + estimPose3d.getY() + ", Z:" + estimPose3d.getZ());
      System.out.println("TImestamp: " + timestamp);
    }
  }

  public boolean filterResults(Pose3d estimPose3d) {

    // If vision’s pose estimation is outside of the field

    // if(estimPose3d.getX()>upperZBound||estimPose3d.getZ()<lowerZBound){ //change if we find out
    // that z starts from camera height
    //     DogLog.log(ntKey + "Filtered because Z is out of bounds: (" + upperZBound + "," +
    // lowerZBound + ")", estimPose3d);
    //     return false;
    // }

    // If vision’s pose estimation is above/below the ground
    double upperZBound = 2.0;
    double lowerZBound = -2.0;
    if (estimPose3d.getZ() > upperZBound
        || estimPose3d.getZ()
            < lowerZBound) { // change if we find out that z starts from camera height
      DogLog.log(
          ntKey + "Filtered because Z is out of bounds: (" + upperZBound + "," + lowerZBound + ")",
          estimPose3d);
      return false;
    }

    return true;
  }

  private Matrix<N3, N1> findSD(
      Optional<EstimatedRobotPose> optionalEstimPose, List<PhotonTrackedTarget> targets) {

    return AprilTagCamConstants.kSingleTagStdDevs;
    //     if(optionalEstimPose.isEmpty()) {
    //         return null;
    //     }
    //     else {
    //         // Pose present. Start running Heuristic
    //         var estStdDevs = AprilTagCamConstants.kSingleTagStdDevs;
    //         int numTags = 0;
    //         double avgDist = 0;

    //         // Precalculation - see how many tags we found, and calculate an average-distance
    // metric
    //         for (var tgt : targets) {
    //             var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
    //             if (tagPose.isEmpty()) continue;
    //             numTags++;
    //             avgDist +=
    //                     tagPose
    //                             .get()
    //                             .toPose2d()
    //                             .getTranslation()
    //
    // .getDistance(optionalEstimPose.get().estimatedPose.toPose2d().getTranslation());
    //         }

    //         if (numTags == 0) {
    //             // No tags visible. Default to single-tag std devs
    //             return null;
    //         } else {
    //             // One or more tags visible, run the full heuristic.
    //             avgDist /= numTags;
    //             // Decrease std devs if multiple targets are visible
    //             if (numTags > 1) estStdDevs = AprilTagCamConstants.kMultiTagStdDevs;
    //             // Increase std devs based on (average) distance
    //             if (numTags == 1 && avgDist > 4)
    //                 estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE,
    // Double.MAX_VALUE);
    //             else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
    //             return estStdDevs;
    //         }

    // }

  }
}
