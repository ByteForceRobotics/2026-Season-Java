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
  SparkMax m_intake;
  SparkMax m_intakeLifter;


  public IntakeSubsystem(){
    m_intake = new SparkMax(IntakeConstants.kIntakeCanId, MotorType.kBrushless);
    m_intakeLifter = new SparkMax(IntakeConstants.kIntakeLifterCanId, MotorType.kBrushless);



    SparkMaxConfig intakeConfig =new SparkMaxConfig();
    SparkMaxConfig intakeLifterConfig =new SparkMaxConfig();
    SoftLimitConfig softLimitConfig = new SoftLimitConfig();
  
    softLimitConfig
      .forwardSoftLimit(0)//positive
      .reverseSoftLimit(-IntakeConstants.kLifterMaxHeight)
      .forwardSoftLimitEnabled(true)
      .reverseSoftLimitEnabled(true);

    intakeConfig
      .smartCurrentLimit(IntakeConstants.kIntakeCurrentLimit);


    intakeLifterConfig
      .apply(softLimitConfig)
      .smartCurrentLimit(IntakeConstants.kIntakeLifterCurrentLimit);
  }
  public void intake(double xSpeed) {
    m_intake.set(xSpeed);
  }

  public void intake_stop() {
    m_intake.set(0.0);

  }
  public void lift(double xSpeed) {
    m_intakeLifter.set(xSpeed);
  }

  public void lift_stop() {
    m_intakeLifter.set(0.0);

  }

  
  @Override
  public void periodic(){
  }
}
