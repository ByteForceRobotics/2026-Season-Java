package frc.robot.subsystems;

import java.io.IOException;
import java.nio.file.Path;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;
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
    PhotonPoseEstimator poseEstimator;
    Optional<EstimatedRobotPose> latestEstimatedPose = Optional.empty();

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

        if (this.tagFieldLayout != null) {
            Transform3d robotToCamera = new Transform3d(
                new Translation3d(this.xOffset, this.yOffset, this.cameraHeight),
                new Rotation3d(0, 0, Math.toRadians(this.camRotation)));
            this.poseEstimator = new PhotonPoseEstimator(
                this.tagFieldLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                robotToCamera);
            this.poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        }
        else {
            this.poseEstimator = null;
        }
    }
    public void refreshCam(){
        var CamResults = camera.getAllUnreadResults();
        if(!CamResults.isEmpty()){
            PhotonPipelineResult CamResult = CamResults.get(CamResults.size() - 1);

            if (this.poseEstimator != null) {
                //this.latestEstimatedPose = this.poseEstimator.update(CamResult);
                this.latestEstimatedPose.ifPresent(est -> this.robotPose = est.estimatedPose.toPose2d());
            }

            if(CamResult.hasTargets()){
                for(int i = 0;i<CamResult.getTargets().size();i++){
                    //List<PhotonTrackedTarget> targetList = CamResult.getTargets();
                    PhotonTrackedTarget target = CamResult.getTargets().get(i);
                    this.targetId = target.getFiducialId();
                    if(this.targetId != 27&&this.targetId != 24&&this.targetId != 11&&this.targetId != 8){continue;}//check which ones this is

                    //all measurements are in degrees
                    this.yaw = target.getYaw();// horizontal  rotation
                    this.pitch = target.getPitch();
                    this.area = target.getArea();
                    //System.out.println(this.yaw+"    "+this.pitch);

                    this.distance = -PhotonUtils.calculateDistanceToTargetMeters(this.cameraHeight, this.targetHeight, 0, Math.toRadians(-this.pitch));
                    
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

    public Optional<EstimatedRobotPose> getLatestEstimatedPose(){
        return this.latestEstimatedPose;
    }
}
