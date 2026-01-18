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
import frc.robot.Constants.IntakeConstants;


public class IntakeSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  SparkMax m_coral;
  SparkMax m_algae;
  DigitalInput m_funnelBeamBreak;
  DigitalInput m_coralBeamBreak;
  boolean funnelBeamBroken;
  boolean coralBeamBroken;


  public IntakeSubsystem(){
    m_coral = new SparkMax(IntakeConstants.kIntakeCanId, MotorType.kBrushless);





    SparkMaxConfig algaeConfig =new SparkMaxConfig();

    algaeConfig
      .smartCurrentLimit(IntakeConstants.kIntakeCurrentLimit);
  }
  public void moveCoral(double xSpeed) {
    m_coral.set(xSpeed);
  }

  public void moveCoral_stop() {
    m_coral.set(0.0);

  }

  
  @Override
  public void periodic(){
  }
}
