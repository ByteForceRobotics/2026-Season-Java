// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SignalsConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.Constants.IntakeConstants;


public class IntakeSubsystem extends SubsystemBase {
  SparkMax m_intake;
  boolean intakeOn = false;
  SysIdRoutine intakeRoutine;

  public IntakeSubsystem(){
    m_intake = new SparkMax(IntakeConstants.kIntakeCanId, MotorType.kBrushless);
    this.intakeRoutine = new SysIdRoutine(new Config(),
      new Mechanism((volts) -> intakeVolts(volts.in(Units.Volts)),
      null,
      this));
    SparkMaxConfig intakeConfig = new SparkMaxConfig();
    
    SignalsConfig signalsConfig = new SignalsConfig();
    signalsConfig
      .primaryEncoderPositionAlwaysOn(true)
      .primaryEncoderVelocityAlwaysOn(true);
        
    intakeConfig
      .apply(signalsConfig)
      .smartCurrentLimit(IntakeConstants.kIntakeCurrentLimit)
      .idleMode(IntakeConstants.kIntakeIdleMode)
      .closedLoop.pid(0.1, 0, 0.1).outputRange(0, 1)
      .allowedClosedLoopError(0, ClosedLoopSlot.kSlot0)//do this finsih, run characterization as well
      .feedForward.sva(0.60703,0.0023506,0.0003995);

     m_intake.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void intake(double xSpeed) {
    if(xSpeed == 0){
      intakeOn = false;
    }
    else{
      intakeOn = true;
    }
    m_intake.set(xSpeed);
  }

  public void intake_stop() {
    intake(0.0);
  }

  public void intake_toggle(){
    System.out.println(" Intake On: " + intakeOn);
    if(intakeOn){
      intake(0);
      intakeOn = false;
    }
    else{
      m_intake.set(IntakeConstants.kIntakeDefaultSpeed);
      intakeOn = true;
    }
  }

  public void intakeVolts(double volts) {
    m_intake.setVoltage(volts);
  }
  public Command intakeCommand(double xSpeed) {
    return this.run(() -> intake(xSpeed)).finallyDo(() -> intake_stop());
  }
  public Command intakeStopCommand() {
    return this.runOnce(() -> intake_stop());
  }
  public Command intakeToggleCommand(){
    return this.runOnce(()->intake_toggle());
  }

  public Command intakeQuasiForward() {return intakeRoutine.quasistatic(SysIdRoutine.Direction.kForward);}

  public Command intakeQuasiReverse() {return intakeRoutine.quasistatic(SysIdRoutine.Direction.kReverse);}

  public Command intakeDynamicForward() {return intakeRoutine.dynamic(SysIdRoutine.Direction.kForward);}

  public Command intakeDynamicReverse() {return intakeRoutine.dynamic(SysIdRoutine.Direction.kReverse);}

  
  @Override
  public void periodic(){
    SmartDashboard.putBoolean("Intake On", intakeOn);
  }
}
