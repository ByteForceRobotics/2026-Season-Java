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
import frc.robot.Constants.IntakeConstants;


public class LifterSubsystem extends SubsystemBase {
  SparkMax m_intakeLifter;

  public LifterSubsystem(){
    m_intakeLifter = new SparkMax(IntakeConstants.kIntakeLifterCanId, MotorType.kBrushless);

    SparkMaxConfig intakeLifterConfig = new SparkMaxConfig();
  
    intakeLifterConfig
      .smartCurrentLimit(IntakeConstants.kIntakeLifterCurrentLimit)
      .idleMode(IntakeConstants.kLifterIdleMode);

     m_intakeLifter.configure(intakeLifterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void lift(double xSpeed) {
    m_intakeLifter.set(xSpeed);
  }

  public void lift_stop() {
    m_intakeLifter.set(0.0);
  }

  public double getLifterPosition() {
    return m_intakeLifter.getAbsoluteEncoder().getPosition();
  }

  public Command liftCommand(double xSpeed) {
    return this.run(() -> lift(xSpeed)).finallyDo(() -> lift_stop());
  }

  public Command liftStopCommand() {
    return this.runOnce(() -> lift_stop());
  }

  @Override
  public void periodic(){
    double lifterPos = m_intakeLifter.getEncoder().getPosition();
    SmartDashboard.putNumber("lifterPos", lifterPos);
  }
}
