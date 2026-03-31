// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import org.photonvision.PhotonCamera;
import org.photonvision.EstimatedRobotPose;
import frc.robot.subsystems.Camera;
import java.util.Optional;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CameraConstants;


public class VisionSubsystem extends SubsystemBase {
  Camera frontCamera;
  Camera rearCamera;
  private final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(0.9, 0.9, 1.8);
  private final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.35, 0.35, 0.9);
  int frontTargetId;
  int rearTargetId;
  double frontyaw;
  double frontpitch;
  double frontarea;

  public VisionSubsystem(){
    frontCamera = new Camera(new PhotonCamera("FrontCam"),CameraConstants.kFrontCamHeight,CameraConstants.kFrontXOffset,CameraConstants.kFrontYOffset,CameraConstants.kFrontRotation);
    rearCamera = new Camera(new PhotonCamera("RearCam"),CameraConstants.kRearCamHeight,CameraConstants.kRearXOffset,CameraConstants.kRearYOffset,CameraConstants.kRearRotation);
    frontTargetId = 0;
    frontyaw = 0;
    frontpitch = 0;
    frontarea = 0;
    rearTargetId = 0;
  }


  @Override
  public void periodic(){
    
    frontCamera.refreshCam();
    //rearCamera.refreshCam();
    
  }
  public Pose2d getPose(){
    return frontCamera.getPose2d();
  }
  public double getDistance(){
    return frontCamera.getDistance();
  }
  public double getPitch(){
    return frontCamera.getPitch();
  }
  public double getYaw(){
    return frontCamera.getYaw();
  }
  public double getHorizDistance(){
    return frontCamera.getHorizDistance();
  }
  public boolean hasTarget(){
    return frontCamera.hasTarget();
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    return frontCamera.getLatestEstimatedPose();
  }

  public Matrix<N3, N1> getEstimationStdDevs(EstimatedRobotPose estimatedPose) {
    int tagCount = estimatedPose.targetsUsed.size();
    if (tagCount <= 0) {
      return kSingleTagStdDevs;
    }

    double avgDistanceMeters = estimatedPose.targetsUsed.stream()
        .mapToDouble(target -> target.getBestCameraToTarget().getTranslation().getNorm())
        .average()
        .orElse(4.0);

    double scale = 1.0 + (avgDistanceMeters * avgDistanceMeters / 25.0);
    Matrix<N3, N1> base = tagCount >= 2 ? kMultiTagStdDevs : kSingleTagStdDevs;

    return VecBuilder.fill(
        base.get(0, 0) * scale,
        base.get(1, 0) * scale,
        base.get(2, 0) * scale);
  }
}
