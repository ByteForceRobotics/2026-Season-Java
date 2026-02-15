package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.proto.Photon;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.Filesystem;
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
        if(!camResults.isEmpty()){
            PhotonPipelineResult camResult = camResults.get(0);
            return m_photonPoseEstimator.update(camResult);
        }
        return null;
    }

}
