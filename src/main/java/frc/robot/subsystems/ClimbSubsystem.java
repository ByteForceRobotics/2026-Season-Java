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

public class ClimbSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  SparkMax m_climber;
  SparkMax m_climber_follower;
  double currentclimbSpeed;

  public ClimbSubsystem(){



    m_climber = new SparkMax(ClimbConstants.kClimbCanId, MotorType.kBrushless);
    m_climber_follower = new SparkMax(ClimbConstants.kClimbFollowerCanId, MotorType.kBrushless);
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
      .follow(m_climber,true);

    m_climber.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_climber_follower.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
  }
  
  /**
   * Method to lift the climb using joystick info.
   *
   */
  
  public void climb(double xSpeed) {
    m_climber.set(xSpeed);
  }
  public double getclimbPosition(){
    return m_climber.getEncoder().getPosition();
  }
  public void climb_stop() {
    if(Math.abs(m_climber.getEncoder().getPosition())<2){
      m_climber.set(0.0);
    }
    else if(Math.abs(m_climber.getEncoder().getPosition())<73){
      m_climber.set(-0.01); 
    }
    else{
      m_climber.set(-0.02);
    }
     //make sure its negative if using passive pwoer
    
  }
  @Deprecated // not working well
  private double calc_speed(double position){
    double distanceToPosition = position-m_climber.getEncoder().getPosition();
    double speed = Math.max(Math.min(0.5,distanceToPosition/10), 0.3);
    if(Math.abs(distanceToPosition)<=0.5){
      speed = 0;
    }
    return speed;
  }
  public void goToPosition(double position){
    double speed = calc_speed(position);
    if(Math.abs(m_climber.getEncoder().getPosition())>Math.abs(position)){
      
    } else {
      speed = -speed;
    }
    m_climber.set(speed);
  }
  public void climbResetEncoders(){
    m_climber.getEncoder().setPosition(0);
    m_climber_follower.getEncoder().setPosition(0);
  }
  
  public void periodic(){

  }
}
