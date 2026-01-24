// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class LauncherSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  SparkMax m_launcher;
  SparkMax m_launcher_follower;
  double currentclimbSpeed;
  boolean extend;

  public LauncherSubsystem(){



    m_launcher = new SparkMax(ClimbConstants.kClimbCanId, MotorType.kBrushless);
    m_launcher_follower = new SparkMax(ClimbConstants.kClimbFollowerCanId, MotorType.kBrushless);
    extend = false;
    SparkMaxConfig globalConfig = new SparkMaxConfig();
    SparkMaxConfig leaderConfig = new SparkMaxConfig();
    SparkMaxConfig followerConfig = new SparkMaxConfig();
    SoftLimitConfig softLimitConfig = new SoftLimitConfig();


    softLimitConfig
      .forwardSoftLimit(0)//positive //need to figure out limits
      .reverseSoftLimit(-ClimbConstants.kClimbLowerLimit)//negative(direction we want to go)
      .forwardSoftLimitEnabled(true)
      .reverseSoftLimitEnabled(true);

    globalConfig
    .apply(softLimitConfig)
      .smartCurrentLimit(ClimbConstants.kClimbCurrentLimit)
      .idleMode(ClimbConstants.kClimbIdleMode);
      
    leaderConfig
      .apply(globalConfig)
      .inverted(false);
      
    followerConfig
      .apply(globalConfig)
      .follow(m_launcher,true);

    m_launcher.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_launcher_follower.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
  }
  
  /**
   * Method to lift the climb using joystick info.
   *
   */
  
  public void launch(double xSpeed) {
    m_launcher.set(xSpeed);
  }
  public void pull_stop() {
    m_launcher.set(0.0);
     //figure out passive power to set
    
  }
  public void periodic(){

  }
}
