// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.AprilTagCam;

import com.ctre.phoenix6.Utils;
import dev.doglog.DogLog;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
  private final Supplier<ChassisSpeeds> currRobotSpeed;
  private AprilTagHelp helper;
  private int counter;

  private final String ntKey;

  Optional<EstimatedRobotPose> optionalEstimPose;

  //
  public AprilTagCam(
      String str,
      Transform3d robotToCam,
      Consumer<AprilTagHelp> addVisionMeasurement,
      Supplier<Pose2d> currRobotPose,
      Supplier<ChassisSpeeds> currRobotSpeed) {
    cam = new PhotonCamera(str);
    this.addVisionMeasurement = addVisionMeasurement;
    this.robotToCam = robotToCam;
    this.currRobotPose = currRobotPose;
    this.currRobotSpeed = currRobotSpeed;


    photonEstimator =
        new PhotonPoseEstimator(
            aprilTagFieldLayout,
            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            robotToCam);

    ntKey = "/Vision/" + str + "/";

    counter = 0;
  }

  public void updatePoseEstim() {

    counter++;

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
    DogLog.log(ntKey + "Number of Results/", results.size());
    DogLog.log(ntKey + "counter", counter);
    if (results.isEmpty()) {
      return;
    }
    for (PhotonPipelineResult targetPose : results) {
      optionalEstimPose = photonEstimator.update(targetPose);


      if (optionalEstimPose.isEmpty()) {
        continue;
      }

      

      Pose3d estimPose3d = optionalEstimPose.get().estimatedPose;

      if(!filterResults(estimPose3d, optionalEstimPose.get())){
        continue; 
      }

      Pose2d pos = estimPose3d.toPose2d(); // yay :0 im so happy
      double timestamp = Utils.fpgaToCurrentTime(targetPose.getTimestampSeconds());
      Matrix<N3, N1> sd = findSD(optionalEstimPose, optionalEstimPose.get().targetsUsed);

      helper = new AprilTagHelp(pos, timestamp, sd);

      DogLog.log(ntKey + "Accepted Pose/", pos);
      DogLog.log(ntKey + "Accepted Time Stamp/", timestamp);
      DogLog.log(ntKey + "Accepted Stdev/", sd);

      addVisionMeasurement.accept(helper);
    }
  }

  public boolean filterResults(Pose3d estimPose3d, EstimatedRobotPose optionalEstimPose) {

    // If vision’s pose estimation is above/below the ground
    double upperZBound = AprilTagCamConstants.Z_TOLERANCE;
    double lowerZBound = -(AprilTagCamConstants.Z_TOLERANCE);
    if (estimPose3d.getZ() > upperZBound
        || estimPose3d.getZ()
            < lowerZBound) { // change if we find out that z starts from camera height
      DogLog.log(
          ntKey + "Filtered because Z is out of bounds: (" + upperZBound + "," + lowerZBound + ")",
          estimPose3d);
      return false;
    }

    //If vision's pose estimation is outside the field
    double upperXBound = AprilTagCamConstants.MAX_X_VALUE + AprilTagCamConstants.XY_TOLERANCE;
    double upperYBound = AprilTagCamConstants.MAX_Y_VALUE + AprilTagCamConstants.XY_TOLERANCE;
    double lowerXYBound = -(AprilTagCamConstants.XY_TOLERANCE);
    if(estimPose3d.getX()< lowerXYBound  || estimPose3d.getY()< lowerXYBound ) {
      DogLog.log(
          ntKey + "Filtered because Y or X is less than 0",
          estimPose3d);
      return false; 
    }
    if(estimPose3d.getX() > upperXBound  || estimPose3d.getY()> upperYBound){
      DogLog.log(
        ntKey + "Filtered because Y or X is out of bounds : (" + upperXBound + "," + upperYBound + "," + lowerXYBound + ")",
        estimPose3d);
    return false; 
    }

    //If the tags are too far away 
    List<PhotonTrackedTarget> trackedTargets = optionalEstimPose.targetsUsed; 
    Transform3d closestTag;
    double min = Double.MAX_VALUE; 
    for(PhotonTrackedTarget currentTarget: trackedTargets){
      if(currentTarget.getBestCameraToTarget().getTranslation().getNorm()< min){
        min = currentTarget.getBestCameraToTarget().getTranslation().getNorm();
        closestTag = currentTarget.getBestCameraToTarget(); 
      } 
    }
    if (min > AprilTagCamConstants.APRILTAG_MAX_DISTANCE){
      return false ; 
    }
  
    // if velocity or rotaion is too high
    double xVel = currRobotSpeed.get().vxMetersPerSecond; 
    double yVel = currRobotSpeed.get().vyMetersPerSecond;
    double vel = Math.sqrt(Math.pow(yVel, 2) + Math.pow(xVel, 2)); 
    double rotation = currRobotSpeed.get().omegaRadiansPerSecond; 

    if(vel > AprilTagCamConstants.MAX_VELOCITY || rotation > AprilTagCamConstants.MAX_ROTATION ){
      return false; 
    }


    return true;
  }

  private Matrix<N3, N1> findSD(
      Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        
    
     if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            return AprilTagCamConstants.kSingleTagStdDevs;

        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = AprilTagCamConstants.kSingleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an average-distance metric
            for (var tgt : targets) {
                var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty()) continue;
                numTags++;
                avgDist +=
                        tagPose
                                .get()
                                .toPose2d()
                                .getTranslation()
                                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                return AprilTagCamConstants.kSingleTagStdDevs;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1) estStdDevs = AprilTagCamConstants.kMultiTagStdDevs;
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else 
                estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                return estStdDevs;
            }
        }

  }
}
