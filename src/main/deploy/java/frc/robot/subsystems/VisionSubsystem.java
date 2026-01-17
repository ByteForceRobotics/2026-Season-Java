// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class VisionSubsystem extends SubsystemBase {
  PhotonCamera frontCamera;
  PhotonCamera rearCamera;
  int frontTargetId;
  int rearTargetId;

  public VisionSubsystem(){
    frontCamera = new PhotonCamera("FrontCam");
    rearCamera = new PhotonCamera("RearCam");
    frontTargetId = 0;
    rearTargetId = 0;
  }


  @Override
  public void periodic(){
    //System.out.println("test");
    var frontCamResults = frontCamera.getAllUnreadResults();
    frontTargetId = 0;
    if(!frontCamResults.isEmpty()){
      PhotonPipelineResult frontCamResult = frontCamResults.get(0);
      if(frontCamResult.hasTargets()){
        //List<PhotonTrackedTarget> targets = frontCamResult.getTargets();
        PhotonTrackedTarget frontTarget = frontCamResult.getBestTarget();
        frontTargetId = frontTarget.getFiducialId();
        //System.out.println(frontTargetId);
      }
    }
    SmartDashboard.putNumber("Front target id",frontTargetId);
    
    var rearCamResults = rearCamera.getAllUnreadResults();
    rearTargetId = 0;
    if(!rearCamResults.isEmpty()){
      PhotonPipelineResult rearCamResult = rearCamResults.get(0);
      if(rearCamResult.hasTargets()){
        //List<PhotonTrackedTarget> targets = rearCamResult.getTargets();
        PhotonTrackedTarget rearTarget = rearCamResult.getBestTarget();
        rearTargetId = rearTarget.getFiducialId();
        System.out.println("rearTargetId: " + rearTargetId);
      }
    }
    SmartDashboard.putNumber("Rear target id",rearTargetId);

  }
}
