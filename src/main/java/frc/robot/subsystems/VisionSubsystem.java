// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import frc.robot.subsystems.Camera;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CameraConstants;


public class VisionSubsystem extends SubsystemBase {
  Camera frontCamera;
  Camera rearCamera;
  int frontTargetId;
  int rearTargetId;
  double frontyaw;
  double frontpitch;
  double frontarea;

  public VisionSubsystem(){
    frontCamera = new Camera(new PhotonCamera("FrontCam"),CameraConstants.kFrontCamHeight);
    rearCamera = new Camera(new PhotonCamera("RearCam"),CameraConstants.kRearCamHeight);
    frontTargetId = 0;
    frontyaw = 0;
    frontpitch = 0;
    frontarea = 0;
    rearTargetId = 0;
  }


  @Override
  public void periodic(){
    frontCamera.refreshCam();
    rearCamera.refreshCam();
    
  }
}
