package frc.robot.subsystems;

import java.io.IOException;
import java.nio.file.Path;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
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
    double targetHeight = 0.55;
    double xOffset;
    double yOffset;
    double camRotation;
    int noTargetCounter = 0;
    Pose2d robotPose;
    AprilTagFieldLayout tagFieldLayout;

    public Camera(PhotonCamera cam, double camHeight,double x,double y, double rot){
        this.camera = cam;
        this.cameraHeight = camHeight;
        this.xOffset = x;
        this.yOffset = y;
        this.camRotation = rot;
        this.robotPose = new Pose2d();
        this.targetId = 0;

        Path path = Filesystem.getDeployDirectory().toPath().resolve("aprilTagFieldLayout.json");
        try{
            this.tagFieldLayout = new AprilTagFieldLayout(path);
        }
        catch(IOException e){
            e.printStackTrace();
            this.tagFieldLayout = null;
        }
    }
    public void refreshCam(){
        var CamResults = camera.getAllUnreadResults();
        if(!CamResults.isEmpty()){
            PhotonPipelineResult CamResult = CamResults.get(0);
            if(CamResult.hasTargets()){
                for(int i = 0;i<CamResult.getTargets().size();i++){
                    //List<PhotonTrackedTarget> targetList = CamResult.getTargets();
                    PhotonTrackedTarget target = CamResult.getTargets().get(i);
                    this.targetId = target.getFiducialId();
                    if(this.targetId != 10&&this.targetId != 26){continue;}//check which ones this is

                    //all measurements are in degrees
                    this.yaw = target.getYaw();// horizontal  rotation
                    this.pitch = target.getPitch();
                    this.area = target.getArea();
                    //System.out.println(this.yaw+"    "+this.pitch);
                    
                    noTargetCounter = 0;
                    Translation2d  camToTargetTranslation = PhotonUtils.estimateCameraToTargetTranslation(this.distance,Rotation2d.fromDegrees(-this.yaw));
                    
                    //fieldToTarget
                    if (this.tagFieldLayout != null) {
                        var optTagPose3d = this.tagFieldLayout.getTagPose(this.targetId);
                        
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
                            this.robotPose = PhotonUtils.estimateFieldToRobot(cameraToTarget, tagPose, cameraToRobot);
                        }
                    }
                    this.distance = -PhotonUtils.calculateDistanceToTargetMeters(this.cameraHeight, this.targetHeight, 0, Math.toRadians(-this.pitch));
                }
            }
        }
        else{
            if(noTargetCounter>=3){
                this.targetId = -2;
                this.yaw = -1;
                this.pitch = -1;
                this.area = -1;
                this.distance=-1;
                this.robotPose = new Pose2d();
            }
            else{
                noTargetCounter++;
            }

        }
        SmartDashboard.putNumber("TagID",this.targetId);
        SmartDashboard.putNumber("Yaw", this.yaw);
        SmartDashboard.putNumber("pitch", this.pitch);
        SmartDashboard.putNumber("area", this.area);
        SmartDashboard.putNumber("distance", this.distance);
        SmartDashboard.putNumber("TagHeight", this.targetHeight);
    }
    public Pose2d getPose2d(){
        return this.robotPose;
    }
    public double getDistance(){
        return this.distance;
    }
    public double getYaw(){
        return this.yaw;
    }
    public double getPitch(){
        return this.pitch;
    }
    public double getHorizDistance(){
        return this.distance*Math.cos(Math.toRadians(this.pitch));
    }
    public String rightOrLeft(){
        return "no";
    }
    public boolean hasTarget(){
        return camera.getLatestResult().hasTargets();
    }
}
