package frc.robot.subsystems.objectDetection;

import java.util.List;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
public class ObjectDetectionCam {

  private final PhotonCamera cam;

  private final Alert visionNotConnected;

    private int counter; 
    private String ntKey;

    public ObjectDetectionCam(String name){
        
        cam = new PhotonCamera(name); 
        counter = 0; 

       ntKey = "/Object Detection/" + name + "/";
    }


    //1. Get all results from the camera
    //2. Check if the list is empty
    //3. If not empty, loop through and getBestTarget
    //4. Get target location in camera space
    //5. Transform (target location in camera space) by (camera location in field space) to get (target location in field space)
    //6. Use three PID controllers x, y, and rotation to drive and rotate robot to target pose
    public void updateDetection(){
        
      counter ++ ; 
      List<PhotonPipelineResult> results = cam.getAllUnreadResults();
      DogLog.log(ntKey + "Number of Results/", results.size());
      DogLog.log(ntKey + "counter", counter);
      
      if(results.isEmpty()){
        return; 
      }

      for(PhotonPipelineResult result: results){
        PhotonTrackedTarget bestTarget = result.getBestTarget(); 
        
      }

    }

    


}