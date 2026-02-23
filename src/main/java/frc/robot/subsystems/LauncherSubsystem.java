// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LauncherConstants;

public class LauncherSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  SparkMax m_launcher1;
  SparkMax m_launcher2;//bottom 

  public LauncherSubsystem(){


    m_launcher1 = new SparkMax(LauncherConstants.kLauncher1CanId, MotorType.kBrushless);//top
    m_launcher2 = new SparkMax(LauncherConstants.kLauncher2CanId, MotorType.kBrushless);//bottom
    SparkMaxConfig globalConfig = new SparkMaxConfig();
    SparkMaxConfig launcher1Config = new SparkMaxConfig();
    SparkMaxConfig launcher2Config = new SparkMaxConfig();

    globalConfig
      .idleMode(LauncherConstants.kLauncherIdleMode);
      
    launcher1Config
    .inverted(true)
      .smartCurrentLimit(LauncherConstants.kLauncher1CurrentLimit);
      
    launcher2Config
    .inverted(true)
      .smartCurrentLimit(LauncherConstants.kLauncher2CurrentLimit);
      

    m_launcher1.configure(launcher1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_launcher2.configure(launcher2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
  }
  
  /**
   * Method to lift the climb using joystick info.
   *
   */
  
  public void launchTop(double xSpeed) {
    m_launcher1.set(-xSpeed);
  }
  public void launchBottom(double xSpeed) {
    m_launcher2.set(xSpeed);
  }
  public void launch_stop() {
    double passivePower = -0.05;//to eject balls when not launching
    m_launcher1.set(passivePower);
    m_launcher2.set(passivePower);
    
  }
  public void launch(double xSpeed) {
    m_launcher1.set(xSpeed);
    m_launcher2.set(0.6);
    
  }
  public void periodic(){

  }
}
