package frc.robot.subsystems.objectDetection;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.aprilTagCam.AprilTagCamConstants;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class ObjectDetectionCam {

  private final PhotonCamera cam;

  private final Supplier<Pose2d> robotPose;
  private int counter;
  private final String ntKey;
  private final Transform3d robotToCam;
  private VisionSystemSim visionSim;
  private final TargetModel simTargetModel;
  private final Pose3d simTargetPose;
  private VisionTargetSim visionTarget;
  private SimCameraProperties cameraProp;
  private PhotonCameraSim cameraSim;

  public ObjectDetectionCam(String name, Transform3d robotToCam, Supplier<Pose2d> robotPose) {

    cam = new PhotonCamera(name);
    counter = 0;
    this.robotPose = robotPose;
    this.robotToCam = robotToCam;
    ntKey = "/Object Detection/" + name + "/";

    visionSim = new VisionSystemSim("main");
    simTargetModel = new TargetModel(0.2);
    simTargetPose =
        new Pose3d(16, 4, 2, new Rotation3d(0, 0, Math.PI)); // placeholder, change later
    visionTarget = new VisionTargetSim(simTargetPose, simTargetModel);
    visionSim.addVisionTargets(visionTarget);
    cameraProp = new SimCameraProperties();
    cameraProp.setCalibration(1280, 800, Rotation2d.fromDegrees(90));
    cameraProp.setFPS(100);
    cameraSim = new PhotonCameraSim(cam, cameraProp);
    visionSim.addCamera(cameraSim, robotToCam);
  }

  // 1. Get all results from the camera
  // 2. Check if the list is empty
  // 3. If not empty, loop through and getBestTarget
  // 4. Get target location in camera space
  // 5. Transform (target location in camera space) by (camera location in field space) to get
  // (target location in field space)
  // 6. Use three PID controllers x, y, and rotation to drive and rotate robot to target pose
  public void updateDetection() {

    counter++;
    List<PhotonPipelineResult> results = cam.getAllUnreadResults();
    DogLog.log(ntKey + "Number of Results/", results.size());
    DogLog.log(ntKey + "counter", counter);

    if (results.isEmpty()) {
      return;
    }

    for (PhotonPipelineResult result : results) {
      PhotonTrackedTarget bestTarget = result.getBestTarget();
      if (bestTarget != null) {
        Transform3d targetLocationToCamera = bestTarget.getBestCameraToTarget();
        Pose3d targetLocationToField = this.getTargetLocInFieldSpace(targetLocationToCamera);

      }
    }
  }

  // Transform (target location in camera space) by (camera location in field space) to get (target
  // location in field space)
  public Pose3d getTargetLocInFieldSpace(Transform3d targetLocationToCamera) {

    Pose2d robotPose = this.robotPose.get();
    Pose3d robotPose3d = new Pose3d(robotPose);
    Pose3d cameraPose3d = robotPose3d.plus(robotToCam);

    Pose3d targetToField = cameraPose3d.plus(targetLocationToCamera);

    DogLog.log(ntKey + "Camera Pose/", cameraPose3d);
    DogLog.log(ntKey + "Target Pose/", targetToField);

    return targetToField;
  }

  public boolean filterResults(
      Pose3d estimPose3d,
      EstimatedRobotPose optionalEstimPose,
      ChassisSpeeds speed){

    // If vision’s pose estimation is above/below the ground
    double upperZBound = AprilTagCamConstants.Z_TOLERANCE;
    double lowerZBound = -(AprilTagCamConstants.Z_TOLERANCE);
    if (estimPose3d.getZ() > upperZBound
        || estimPose3d.getZ()
            < lowerZBound) { // change if we find out that z starts from camera height
      DogLog.log(ntKey + "Rejected Pose", estimPose3d);
      DogLog.log(ntKey + "Rejected Reason", "out of Z bounds", "Z: " + estimPose3d.getZ());
      return false;
    }


  // If vision's pose estimation is outside the field
  double upperXBound = AprilTagCamConstants.MAX_X_VALUE + AprilTagCamConstants.XY_TOLERANCE;
  double upperYBound = AprilTagCamConstants.MAX_Y_VALUE + AprilTagCamConstants.XY_TOLERANCE;
  double lowerXYBound = -(AprilTagCamConstants.XY_TOLERANCE);
  if (estimPose3d.getX() < lowerXYBound || estimPose3d.getY() < lowerXYBound) {
    DogLog.log(ntKey + "Rejected Pose", estimPose3d);
    DogLog.log(ntKey + "Rejected Reason", "Y or X is less than 0");
    return false;
  }
  if (estimPose3d.getX() > upperXBound || estimPose3d.getY() > upperYBound) {
    DogLog.log(ntKey + "Rejected Pose", estimPose3d);
    DogLog.log(
        ntKey + "Rejected Reason",
        "Y or X is out of bounds",
        "X: " + estimPose3d.getX() + "," + "Y: " + estimPose3d.getX());

    return false;
  }

         // if velocity or rotaion is too high
    double xVel = speed.vxMetersPerSecond;
    double yVel = speed.vyMetersPerSecond;
    double vel = Math.sqrt(Math.pow(yVel, 2) + Math.pow(xVel, 2));
    double rotation = speed.omegaRadiansPerSecond;

    if (vel > AprilTagCamConstants.MAX_VELOCITY || rotation > AprilTagCamConstants.MAX_ROTATION) {
      DogLog.log(ntKey + "Rejected Pose", estimPose3d);
      DogLog.log(ntKey + "Rejected Reason", "Velocity/Rotation is too fast");
      return false;
    }

        return true; 
      }
 
}
