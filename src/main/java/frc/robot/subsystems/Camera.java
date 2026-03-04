package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Transform2d;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
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
    double targetHeight = 1;
    double xOffset;
    double yOffset;
    double camRotation;
    Pose2d robotPose;
    
    public Camera(PhotonCamera cam, double camHeight,double x,double y, double rot){
        this.camera = cam;
        this.cameraHeight = camHeight;
        this.xOffset = x;
        this.yOffset = y;
        this.camRotation = rot;
        this.robotPose = new Pose2d();
        this.targetId = 0;

    }
    public void refreshCam(){
    var CamResults = camera.getAllUnreadResults();
        
        if(!CamResults.isEmpty()){
            PhotonPipelineResult CamResult = CamResults.get(0);
            if(CamResult.hasTargets()){
                //List<PhotonTrackedTarget> targetList = CamResult.getTargets();
                PhotonTrackedTarget target = CamResult.getBestTarget();
                this.targetId = target.getFiducialId();
                this.yaw = target.getYaw();// horizontal  rotation
                this.pitch = target.getPitch();
                this.area = target.getArea();
                System.out.println(this.yaw+"    "+this.pitch);
                //camToTargetTranslation
                this.distance = PhotonUtils.calculateDistanceToTargetMeters(this.cameraHeight, this.targetHeight, 0, Math.toRadians(-this.pitch));
    
                Translation2d  camToTargetTranslation = PhotonUtils.estimateCameraToTargetTranslation(this.distance,Rotation2d.fromDegrees(-this.yaw));
                //fieldToTarget
                try{
                    
                    Path path  = Filesystem.getDeployDirectory().toPath().resolve("aprilTagFieldLayout.json");
                    AprilTagFieldLayout tagFieldLayout = new AprilTagFieldLayout(path);
                    var optTagPose3d = tagFieldLayout.getTagPose(this.targetId);
                    if (optTagPose3d.isPresent()) {
                        var tagPose3d = optTagPose3d.get();
                        Pose2d tagPose = tagPose3d.toPose2d();
                        this.targetHeight = tagPose3d.getZ();

                        //gyroAngle
                        Rotation2d gyroAngle = Rotation2d.fromDegrees(0);//set this
                        
                        //cameraToTarget
                        Transform2d cameraToTarget = PhotonUtils.estimateCameraToTarget(camToTargetTranslation, tagPose, gyroAngle);
                        
                        //cameraTransform2d
                        Translation2d translation = new Translation2d(this.xOffset, this.yOffset);
                        // Create a rotation
                        Rotation2d rotation = Rotation2d.fromDegrees(this.camRotation);
                        
                        // Create the transform
                        Transform2d cameraToRobot = new Transform2d(translation, rotation);

                        //finalRobotPose
                        this.robotPose = PhotonUtils.estimateFieldToRobot(cameraToTarget,tagPose,cameraToRobot);
                    }
                }
                catch(IOException e){
                    this.targetId = 1;
                    this.yaw = 1;
                    this.pitch = 1;
                    this.area = 1;
                    this.distance=1;
                    this.robotPose = new Pose2d();
                }

            }
            else{
                this.targetId = -2;
                this.yaw = -1;
                this.pitch = -1;
                this.area = -1;
                this.distance=-1;
                this.robotPose = new Pose2d();
            }
        }
        SmartDashboard.putNumber(this.camera.getName(),this.targetId);
        SmartDashboard.putNumber("Yaw", this.yaw);
        SmartDashboard.putNumber("pitch", this.pitch);
        SmartDashboard.putNumber("area", this.area);
        SmartDashboard.putNumber("distance", this.distance);
    }
    public Pose2d getPose2d(){
        return this.robotPose;
    }
}
