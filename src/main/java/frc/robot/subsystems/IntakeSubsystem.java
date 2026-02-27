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

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.IntakeConstants;


public class IntakeSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  SparkMax m_intake;
  SparkMax m_intakeLifter;
  boolean intakeOn;


  public IntakeSubsystem(){
    m_intake = new SparkMax(IntakeConstants.kIntakeCanId, MotorType.kBrushless);
    m_intakeLifter = new SparkMax(IntakeConstants.kIntakeLifterCanId, MotorType.kBrushless);
    intakeOn = false;

    SparkMaxConfig intakeConfig = new SparkMaxConfig();
    SparkMaxConfig intakeLifterConfig = new SparkMaxConfig();
    SoftLimitConfig softLimitConfig = new SoftLimitConfig();
  
    softLimitConfig
      .forwardSoftLimit(IntakeConstants.kLifterMaxHeight)//positive
      .reverseSoftLimit(0)
      .forwardSoftLimitEnabled(false)
      .reverseSoftLimitEnabled(false);

    intakeConfig
      .smartCurrentLimit(IntakeConstants.kIntakeCurrentLimit)
      .idleMode(IntakeConstants.kIntakeIdleMode);


    intakeLifterConfig
      .apply(softLimitConfig)
      .smartCurrentLimit(IntakeConstants.kIntakeLifterCurrentLimit)
      .idleMode(IntakeConstants.kLifterIdleMode);

     m_intake.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
     m_intakeLifter.configure(intakeLifterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
  public void intake(double xSpeed) {
    m_intake.set(xSpeed);
  }

  public void intake_stop() {
    m_intake.set(0.0);

  }
  public void intake_toggle(){
    if(intakeOn){
      intakeOn = false;
      m_intake.set(0);
    }
    else{
      intakeOn = true;
      m_intake.set(IntakeConstants.kIntakeDefaultSpeed);
    }
  }
  public void lift(double xSpeed) {
    m_intakeLifter.set(xSpeed);
  }

  public void lift_stop() {
    if(m_intakeLifter.getEncoder().getPosition() >3.9){
      double passivePower = 0.05;//to hold the lifter down when intaking
      m_intakeLifter.set(passivePower);
    }
    else if(m_intakeLifter.getEncoder().getPosition() <0.1){
      m_intakeLifter.set(-0.01);
    }
    else{
      m_intakeLifter.set(0.0);
    }

  }

  
  @Override
  public void periodic(){
    double lifterPos = m_intakeLifter.getEncoder().getPosition();
    SmartDashboard.putNumber("lifterPos", lifterPos);
  }
}
