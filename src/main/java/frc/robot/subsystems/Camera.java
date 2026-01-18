package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Camera {
    PhotonCamera camera;
    int targetId;
    double yaw;
    double pitch;
    double area;
    public Camera(PhotonCamera cam){
        this.camera = cam;

    }
    public void refreshCam(){
    var CamResults = camera.getAllUnreadResults();
        
        if(!CamResults.isEmpty()){
            PhotonPipelineResult CamResult = CamResults.get(0);
            if(CamResult.hasTargets()){
                //List<PhotonTrackedTarget> targetList = CamResult.getTargets();
                PhotonTrackedTarget target = CamResult.getBestTarget();
                targetId = target.getFiducialId();
                this.yaw = target.getYaw();
                this.pitch = target.getPitch();
                this.area = target.getArea();
                //Transform2d pose = target.getCameraToTarget();
            }
            else{
                this.targetId = 0;
                this.yaw = 0;
                this.pitch = 0;
                this.area = 0;
            }
        }
        SmartDashboard.putNumber("Front target id",targetId);
        SmartDashboard.putNumber("Yaw", yaw);
        SmartDashboard.putNumber("pitch", pitch);
        SmartDashboard.putNumber("area", area);
    }
}
