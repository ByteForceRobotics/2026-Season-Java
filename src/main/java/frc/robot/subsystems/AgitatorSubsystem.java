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
import frc.robot.Constants.IntakeConstants;


public class AgitatorSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  SparkMax m_agitator;
  SparkMax m_agitatorLifter;
  public boolean agitateVar = false;

  public AgitatorSubsystem(){
    m_agitator = new SparkMax(AgitatorConstants.kAgitatorCanId, MotorType.kBrushed);

    SparkMaxConfig agitatorConfig = new SparkMaxConfig();

    agitatorConfig
      .smartCurrentLimit(AgitatorConstants.kAgitatorCurrentLimit);

     m_agitator.configure(agitatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
  public void agitate(double xSpeed) {
    m_agitator.set(xSpeed);
    if (xSpeed >0){
      agitateVar = true;
    }
    else{
      agitateVar = false;
    }
  }

  public void agitate_stop(double xSpeed) {
    agitate(0);
  }

  public void agitate_toggle(){
    if(agitateVar == true){
      agitate_stop(AgitatorConstants.kAgitatorDefaultSpeed);
    }
    else{
      agitate(AgitatorConstants.kAgitatorDefaultSpeed);
    }
  }
  public Command agitateCommand(double xSpeed) {
    return this.runOnce(() -> agitate(xSpeed));
  }
  public Command agitateStopCommand(double xSpeed) {
    return this.runOnce(() -> agitate_stop(xSpeed));
  }
  public Command agitateToggleCommand() {
    return this.runOnce(() -> agitate_toggle());
  }
  
  @Override
  public void periodic(){
    SmartDashboard.putBoolean("agitator enabled", agitateVar);
  }
}
