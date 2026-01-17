// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  SparkMax m_elevator;
  SparkMax m_elevator_follower;
  double goalPos;
  double currentElevatorSpeed;
  String currentConfig;
  double prevPos = 0;
  PIDController pid = new PIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);

  public ElevatorSubsystem(){
    pid.setTolerance(1);



    m_elevator = new SparkMax(ElevatorConstants.kElevatorCanId, MotorType.kBrushless);
    m_elevator_follower = new SparkMax(ElevatorConstants.kElevatorFollowerCanId, MotorType.kBrushless);
    SparkMaxConfig globalConfig = new SparkMaxConfig();
    SparkMaxConfig leaderConfig = new SparkMaxConfig();
    SparkMaxConfig followerConfig = new SparkMaxConfig();
    SoftLimitConfig softLimitConfig = new SoftLimitConfig();
    currentConfig = "default";


    softLimitConfig
      .forwardSoftLimit(0)//positive
      .reverseSoftLimit(-ElevatorConstants.maxNeededElevatorHeight)//negative(direction we want to go)
      .forwardSoftLimitEnabled(true)
      .reverseSoftLimitEnabled(true);

    globalConfig
    .apply(softLimitConfig)
      .smartCurrentLimit(ElevatorConstants.kElevatorCurrentLimit)
      .idleMode(ElevatorConstants.kElevatorIdleMode);
      
    leaderConfig
      .apply(globalConfig)
      .inverted(false);
      
    followerConfig
      .apply(globalConfig)
      .follow(m_elevator,true);

    m_elevator.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_elevator_follower.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
  }
  
  /**
   * Method to lift the elevator using joystick info.
   *
   */
  
  public void lift(double xSpeed) {
    double truexSpeed = xSpeed;
    
    if(Math.abs(m_elevator.getEncoder().getPosition()+150)<15){
      truexSpeed = xSpeed/4;
    }
    else if(Math.abs(m_elevator.getEncoder().getPosition())<15){
      truexSpeed = xSpeed/4;
    }
    else if(Math.abs(m_elevator.getEncoder().getPosition()+75)<15){
      truexSpeed = xSpeed/2;
    }
    else{
      truexSpeed = xSpeed;
    }
    m_elevator.set(truexSpeed);
  }
  public double getElevatorPosition(){
    return m_elevator.getEncoder().getPosition();
  }
  public void setGoalPosition(){
    goalPos = m_elevator.getEncoder().getPosition();
  }
  public void lift_stop() {
    if(Math.abs(m_elevator.getEncoder().getPosition())<2){
      m_elevator.set(0.0);
    }
    else if(Math.abs(m_elevator.getEncoder().getPosition())<73){
      m_elevator.set(-0.01); 
    }
    else{
      m_elevator.set(-0.02);
    }
     //make sure its negative if using passive pwoer
    
  }
  @Deprecated // not working well
  public void goToPIDPosition(double position){
    m_elevator.set(pid.calculate(m_elevator.getEncoder().getPosition(), position));
    if(pid.atSetpoint()){
      pid.reset();
    }
    if(prevPos!= position){
      pid.reset();
    }
    prevPos = position;
  }
  private double calc_speed(double position){
    double distanceToPosition = position-m_elevator.getEncoder().getPosition();
    double speed = Math.max(Math.min(0.5,distanceToPosition/10), 0.3);
    if(Math.abs(distanceToPosition)<=0.5){
      speed = 0;
    }
    return speed;
  }
  public void goToPosition(double position){
    double speed = calc_speed(position);
    if(Math.abs(m_elevator.getEncoder().getPosition())>Math.abs(position)){
      
    } else {
      speed = -speed;
    }
    m_elevator.set(speed);
  }
  public void elevatorResetEncoders(){
    m_elevator.getEncoder().setPosition(0);
    m_elevator_follower.getEncoder().setPosition(0);
  }
  public void stayAtPosition(){
    if( m_elevator.getEncoder().getVelocity()<0.1){
      goToPosition(goalPos);
    }
  }
  public void ElevatorFixModeEnable(){
    currentConfig = "fixmode";
    SparkMaxConfig globalElevatorFixConfig =new SparkMaxConfig();
    SparkMaxConfig leaderElevatorFixConfig = new SparkMaxConfig();
    SparkMaxConfig followerElevatorFixConfig = new SparkMaxConfig();
    SoftLimitConfig ElevatorFixLimitConfig = new SoftLimitConfig();

    ElevatorFixLimitConfig
      .forwardSoftLimit(0)//positive
      .reverseSoftLimit(-ElevatorConstants.maxNeededElevatorHeight)//negative(direction we want to go)
      .forwardSoftLimitEnabled(false)
      .reverseSoftLimitEnabled(false);

    globalElevatorFixConfig
    .apply(ElevatorFixLimitConfig)
      .smartCurrentLimit(ElevatorConstants.kElevatorCurrentLimit)
      .idleMode(ElevatorConstants.kElevatorIdleMode);
    
    leaderElevatorFixConfig
      .apply(globalElevatorFixConfig)
      .inverted(false);
      
    followerElevatorFixConfig
      .apply(globalElevatorFixConfig)
      .follow(m_elevator,true);
    m_elevator.configure(leaderElevatorFixConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_elevator_follower.configure(followerElevatorFixConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
  public void ElevatorFixModeDisable(){

    SparkMaxConfig globalConfig = new SparkMaxConfig();
    SparkMaxConfig leaderConfig = new SparkMaxConfig();
    SparkMaxConfig followerConfig = new SparkMaxConfig();
    SoftLimitConfig softLimitConfig = new SoftLimitConfig();
    currentConfig = "default";


    softLimitConfig
      .forwardSoftLimit(0)//positive
      .reverseSoftLimit(-ElevatorConstants.maxNeededElevatorHeight)//negative(direction we want to go)
      .forwardSoftLimitEnabled(true)
      .reverseSoftLimitEnabled(true);

    globalConfig
    .apply(softLimitConfig)
      .smartCurrentLimit(ElevatorConstants.kElevatorCurrentLimit)
      .idleMode(ElevatorConstants.kElevatorIdleMode);
      
    leaderConfig
      .apply(globalConfig)
      .inverted(false);
      
    followerConfig
      .apply(globalConfig)
      .follow(m_elevator,true);
      
    m_elevator.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_elevator_follower.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
  }
  public void toggleElevatorFixMode(){
    if(currentConfig.equals("default")){
      ElevatorFixModeEnable();
    }
    else{
      ElevatorFixModeDisable();
    }
  }
  @Override
  public void periodic(){
    SmartDashboard.putString("Current Config", currentConfig);
    SmartDashboard.putNumber("Elevator Speed", m_elevator.getEncoder().getVelocity());
    SmartDashboard.putNumber("Elevator pos", m_elevator.getEncoder().getPosition());
    SmartDashboard.putNumber("Elevator Goal Pos", goalPos);
  }
}
