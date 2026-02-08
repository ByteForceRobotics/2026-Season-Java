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
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.LauncherConstants;

public class LauncherSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  SparkMax m_launcher1;
  SparkMax m_launcher2;

  public LauncherSubsystem(){



    m_launcher1 = new SparkMax(ClimbConstants.kClimbCanId, MotorType.kBrushless);
    m_launcher2 = new SparkMax(ClimbConstants.kClimbFollowerCanId, MotorType.kBrushless);
    SparkMaxConfig globalConfig = new SparkMaxConfig();
    SparkMaxConfig launcher1Config = new SparkMaxConfig();
    SparkMaxConfig launcher2Config = new SparkMaxConfig();

    globalConfig
      .idleMode(LauncherConstants.kLauncherIdleMode);
      
    launcher1Config
      .smartCurrentLimit(LauncherConstants.kLauncher1CurrentLimit);
      
    launcher2Config
      .smartCurrentLimit(LauncherConstants.kLauncher2CurrentLimit);

    m_launcher1.configure(launcher1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_launcher2.configure(launcher2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
  }
  
  /**
   * Method to lift the climb using joystick info.
   *
   */
  
  public void launch(double xSpeed) {
    m_launcher1.set(xSpeed);
    m_launcher2.set(xSpeed/2);
  }
  public void launch_stop() {
    m_launcher1.set(0.0);
    m_launcher2.set(0.0);
    
  }
  public void periodic(){

  }
}
