package frc.robot.subsystems;

import java.io.IOException;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.math.geometry.Transform2d;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Camera 
{
    PhotonCamera camera;
    int targetId;
    double yaw;
    double pitch;
    double area;
    double cameraHeight;
    double distance;
    double targetHeight = 0;
    double xOffset;
    double yOffset;
    double rotation;
    Pose2d robotPose;
    public Camera(PhotonCamera cam, double camHeight,double x,double y, double rot){
        this.camera = cam;
        this.cameraHeight = camHeight;
        this.xOffset = x;
        this.yOffset = y;
        this.rotation = rot;
        this.robotPose = new Pose2d();

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
                //camToTargetTranslation
                double distToTarget = PhotonUtils.calculateDistanceToTargetMeters(cameraHeight, targetHeight, 0, pitch);
                Translation2d  camToTargetTranslation = PhotonUtils.estimateCameraToTargetTranslation(distToTarget,Rotation2d.fromDegrees(-yaw));
                //fieldToTarget
                try{
                    AprilTagFieldLayout tagFieldLayout = new AprilTagFieldLayout("..\\2026 Season Java\\aprilTagFieldLayout.json");
                    var optTagPose3d = tagFieldLayout.getTagPose(targetId);
                    if (optTagPose3d.isPresent()) {
                        var tagPose3d = optTagPose3d.get();
                        Pose2d tagPose = tagPose3d.toPose2d();
                        targetHeight = tagPose3d.getZ();

                        //gyroAngle
                        Rotation2d gyroAngle = Rotation2d.fromDegrees(0);//set this
                        
                        //cameraToTarget
                        Transform2d cameraToTarget = PhotonUtils.estimateCameraToTarget(camToTargetTranslation, tagPose, gyroAngle);
                        
                        //cameraTransform2d
                            Translation2d translation = new Translation2d(this.xOffset, this.yOffset);
                            // Create a rotation
                            Rotation2d rotation = Rotation2d.fromDegrees(this.rotation);
                            
                        // Create the transform
                        Transform2d cameraToRobot = new Transform2d(translation, rotation);

                        //finalRobotPose
                        robotPose = PhotonUtils.estimateFieldToRobot(cameraToTarget,tagPose,cameraToRobot);
                    }
                }
                catch(IOException e){
                    this.targetId = 0;
                    this.yaw = 0;
                    this.pitch = 0;
                    this.area = 0;
                    this.distance=0;
                }

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
        SmartDashboard.putNumber("distance", distance);
    }
    
}
