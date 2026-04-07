// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AgitatorConstants;


public class AgitatorSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  SparkMax m_agitatorMain;
  SparkMax m_agitatorIntake;
  public boolean agitateMainVar = false;
  public boolean agitateIntakeVar = false;
  public double agitatorIntakeMultiplier = 1.05;

  public AgitatorSubsystem(){
    m_agitatorMain = new SparkMax(AgitatorConstants.kAgitatorMainCanId, MotorType.kBrushless);
    m_agitatorIntake = new SparkMax(AgitatorConstants.kAgitatorIntakeCanId, MotorType.kBrushless);
    SparkMaxConfig agitatorConfig = new SparkMaxConfig();

    agitatorConfig
      .smartCurrentLimit(AgitatorConstants.kAgitatorCurrentLimit);

     m_agitatorMain.configure(agitatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
     m_agitatorIntake.configure(agitatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
  public void agitateMain(double xSpeed) {
    m_agitatorMain.set(xSpeed);
    if (xSpeed >0){
      agitateMainVar = true;
    }
    else{
      agitateMainVar = false;
    }
  }
  public void agitateIntake(double xSpeed) {
    m_agitatorIntake.set(xSpeed*agitatorIntakeMultiplier);
    if (xSpeed >0){
      agitateIntakeVar = true;
    }
    else{
      agitateIntakeVar = false;
    }
  }
  public void agitateMain(){
    agitateMain(AgitatorConstants.kAgitatorDefaultSpeed);
  }
  public void agitateIntake(){
    agitateIntake(AgitatorConstants.kAgitatorDefaultSpeed*agitatorIntakeMultiplier);//idk what times we need to do
  }

  public void agitateMain_stop(double xSpeed) {
    agitateMain(0);
  }
  public void agitateIntake_stop(double xSpeed) {
    agitateIntake(0);
  }
  public void agitateMain_stop(){
    agitateMain(0);
  }
  public void agitateIntake_stop(){
    agitateIntake(0);
  }

  public void agitateMain_toggle(){
    if(agitateMainVar == true){
      agitateMain_stop();
    }
    else{
      agitateMain(AgitatorConstants.kAgitatorDefaultSpeed);
    }
  }
  public void agitateIntake_toggle(){
    if(agitateIntakeVar == true){
      agitateIntake_stop();
    }
    else{
      agitateIntake(AgitatorConstants.kAgitatorDefaultSpeed*agitatorIntakeMultiplier);
    }
  }
  public void stopAll(){
    agitateMain_stop();
    agitateIntake_stop();
  }
  public Command stopAllCommand(){
    return this.runOnce(() -> stopAll());
  }

  public Command agitateMainCommand(double xSpeed) {
    return this.run(() -> agitateMain(xSpeed)).finallyDo(() -> agitateMain_stop(xSpeed));
  }

  public Command agitateIntakeCommand(double xSpeed) {
    return this.run(() -> agitateIntake(xSpeed)).finallyDo(() -> agitateIntake_stop(xSpeed));
  }
  public Command agitateMainStopCommand(double xSpeed) {
    return this.runOnce(() -> agitateMain_stop(xSpeed));
  }
  public Command agitateIntakeStopCommand(double xSpeed) {
    return this.runOnce(() -> agitateIntake_stop(xSpeed));
  }
  public Command agitateMainToggleCommand() {
    return this.runOnce(() -> agitateMain_toggle());
  }
  public Command agitateIntakeToggleCommand() {
    return this.runOnce(() -> agitateIntake_toggle());
  }
  
  @Override
  public void periodic(){
    SmartDashboard.putBoolean("agitatorMain enabled", agitateMainVar);
    SmartDashboard.putBoolean("agitatorIntake enabled", agitateIntakeVar);
  }
  
}
