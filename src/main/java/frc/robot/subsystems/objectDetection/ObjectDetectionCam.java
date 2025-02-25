package frc.robot.subsystems.objectDetection;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.List;
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class ObjectDetectionCam {

  private final PhotonCamera cam;

  private final Supplier<Pose2d> robotPose;
  private int counter;
  private final String ntKey;
  private final Transform3d robotToCam;

  public ObjectDetectionCam(String name, Transform3d robotToCam, Supplier<Pose2d> robotPose) {

    cam = new PhotonCamera(name);
    counter = 0;
    this.robotPose = robotPose;
    this.robotToCam = robotToCam;
    ntKey = "/Object Detection/" + name + "/";
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
}
