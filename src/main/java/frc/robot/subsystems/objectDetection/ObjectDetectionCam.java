package frc.robot.subsystems.objectDetection;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.List;
import java.util.function.Supplier;
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
        if (!filterResults(targetLocationToField)) {
          break;
        }
        DogLog.log(ntKey + "Accepted Target Pose/", targetLocationToField);
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

    DogLog.log(ntKey + "Camera Pose/", targetLocationToCamera);
    return targetToField;
  }

  public boolean filterResults(Pose3d detectedTargetPose) {

    // If vision’s detected target is below the ground/above tolerable height
    double upperZBound = ObjectDetectionConstants.UPPER_Z_TOLERANCE;
    double lowerZBound = ObjectDetectionConstants.LOWER_Z_TOLERANCE;
    if (detectedTargetPose.getZ() > upperZBound
        || detectedTargetPose.getZ()
            < lowerZBound) { // change if we find out that z starts from camera height
      DogLog.log(ntKey + "Rejected Target Pose", detectedTargetPose);
      DogLog.log(
          ntKey + "Rejected Reason", "out of Z bounds", "Z: " + detectedTargetPose.getZ());
      return false;
    }

    // If vision's detected target pose is outside the field
    double upperXBound =
        ObjectDetectionConstants.MAX_X_VALUE + ObjectDetectionConstants.XY_TOLERANCE;
    double upperYBound =
        ObjectDetectionConstants.MAX_Y_VALUE + ObjectDetectionConstants.XY_TOLERANCE;
    double lowerXYBound = -(ObjectDetectionConstants.XY_TOLERANCE);
    if (detectedTargetPose.getX() < lowerXYBound
        || detectedTargetPose.getY() < lowerXYBound) {
      DogLog.log(ntKey + "Rejected Target Pose", detectedTargetPose);
      DogLog.log(ntKey + "Rejected Reason", "Y or X is less than 0");
      return false;
    }
    if (detectedTargetPose.getX() > upperXBound || detectedTargetPose.getY() > upperYBound) {
      DogLog.log(ntKey + "Rejected Target Pose", detectedTargetPose);
      DogLog.log(
          ntKey + "Rejected Reason",
          "Y or X is out of bounds",
          "X: " + detectedTargetPose.getX() + "," + "Y: " + detectedTargetPose.getX());

      return false;
    }

    return true;
  }
}
