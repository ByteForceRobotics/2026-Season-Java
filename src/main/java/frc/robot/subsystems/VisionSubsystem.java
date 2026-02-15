// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import frc.robot.subsystems.Camera;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CameraConstants;


public class VisionSubsystem extends SubsystemBase {
  Camera frontCamera;
  Camera rearCamera;
  AprilTagFieldLayout m_fieldLayout;

  public VisionSubsystem(){
    Path path = Filesystem.getDeployDirectory().toPath().resolve("aprilTagFieldLayout.json");
        try{
            m_fieldLayout = new AprilTagFieldLayout(path);
            frontCamera = new Camera("FrontCam",m_fieldLayout);
            rearCamera = new Camera("RearCam",m_fieldLayout);
        }
        catch(IOException e){}
    
  }
  public Optional<EstimatedRobotPose> getEstimatedGlobalPoseFront(){
    return frontCamera.getEstimatedGlobalPose();
  }

  @Override
  public void periodic(){
    
  }
}
