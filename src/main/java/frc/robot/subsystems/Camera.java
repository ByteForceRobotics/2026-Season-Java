package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Camera {
    PhotonCamera camera;
    int targetId;
    double yaw;
    double pitch;
    double area;
    double cameraHeight;
    double distance;
    double targetHeight = 0;
    public Camera(PhotonCamera cam, double camHeight){
        this.camera = cam;
        this.cameraHeight = camHeight;

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
                this.distance = PhotonUtils.estimateFieldToRobot(cameraHeight, targetHeight, 0, pitch,Rotation2d.fromDegrees(-target.getYaw()));
                //camToTargetTranslation
                double distToTarget = PhotonUtils.calculateDistanceToTargetMeters(cameraHeight, targetHeight, 0, pitch);
                Translation2d  camToTargetTranslation = PhotonUtils.estimateCameraToTargetTranslation(distToTarget,Rotation2d.fromDegrees(-yaw));

                //fieldToTarget
                AprilTagFieldLayout tagFieldLayout = new AprilTagFieldLayout("C:\FRC Code\2026 Season Java\2026 Season Java\aprilTagFieldLayout.json");
                tagFieldLayout.getTagPose(targetId);
                

                PhotonUtils.estimateCameraToTarget(camToTargetTranslation, null, null);
                //Transform2d pose = target.getCameraToTarget();
            }
            else{
                this.targetId = 0;
                this.yaw = 0;
                this.pitch = 0;
                this.area = 0;
                this.distance=0;
            }
        }
        SmartDashboard.putNumber("Front target id",targetId);
        SmartDashboard.putNumber("Yaw", yaw);
        SmartDashboard.putNumber("pitch", pitch);
        SmartDashboard.putNumber("area", area);
    }
}
