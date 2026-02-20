package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Camera 
{
    PhotonCamera m_camera;
    private final AprilTagFieldLayout m_fieldLayout;
    private final Transform3d m_robotToCam = new Transform3d(
        new Translation3d(0.3,0.0,0.5),
        new Rotation3d(0, Math.toRadians(-15),0)//set these
    );
    
    private final PhotonPoseEstimator m_photonPoseEstimator;

    public Camera(String name,AprilTagFieldLayout fieldLayout){
        m_camera = new PhotonCamera(name);
        this.m_fieldLayout = fieldLayout;

        m_photonPoseEstimator = new PhotonPoseEstimator(
            m_fieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            m_robotToCam);

    }
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(){
        List<PhotonPipelineResult> camResults = m_camera.getAllUnreadResults();
        for (var result:camResults){
            var  multiTagResult = result.getMultiTagResult();
            if(multiTagResult.isPresent()){
                var fieldToCamera = multiTagResult.get().estimatedPose.best;
                SmartDashboard.putString("estimatedpose",fieldToCamera.toString());
            }
        }
        return Optional.ofNullable(null);
        
        //SmartDashboard.putNumber("null", 10);
        // if(!camResults.isEmpty()){
        //     SmartDashboard.putNumber("nulltest", 99);
        //     PhotonPipelineResult camResult = camResults.get(0);
        //     PhotonTrackedTarget tag = new PhotonTrackedTarget();
        //     if(camResult.getBestTarget()!=null){
        //         SmartDashboard.putNumber("tag1", camResult.getBestTarget().getFiducialId());
        //     }
        //     SmartDashboard.putNumber("tag1", -1);
        //     Optional<EstimatedRobotPose> camPose = m_photonPoseEstimator.update(camResult);
        //     SmartDashboard.putString("key3",camPose.toString());
        //     return camPose;
        // 
    }

}
