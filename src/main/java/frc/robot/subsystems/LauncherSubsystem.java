// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LauncherConstants;


public class LauncherSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  SparkMax m_launcher;


  public LauncherSubsystem(){
    m_launcher = new SparkMax(LauncherConstants.kLauncherCanId, MotorType.kBrushless);





    SparkMaxConfig LauncherConfig =new SparkMaxConfig();

    LauncherConfig
      .smartCurrentLimit(LauncherConstants.kLauncherCurrentLimit);
  }
  public void launch(double xSpeed) {
    m_launcher.set(xSpeed);
  }

  public void launchStop() {
    m_launcher.set(0.0);

  }

  
  @Override
  public void periodic(){
  }
}
