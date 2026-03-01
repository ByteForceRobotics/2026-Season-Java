// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LauncherConstants;

public class LauncherSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  SparkMax m_launcher1; //top flywheel
  SparkMax m_launcher2; //bottom 

  public LauncherSubsystem(){
    


    m_launcher1 = new SparkMax(LauncherConstants.kLauncher1CanId, MotorType.kBrushless);//top
    m_launcher2 = new SparkMax(LauncherConstants.kLauncher2CanId, MotorType.kBrushless);//bottom
    SparkMaxConfig launcher1Config = new SparkMaxConfig();
    SparkMaxConfig launcher2Config = new SparkMaxConfig();
      
    launcher1Config
    .inverted(true)
    .idleMode(LauncherConstants.kLauncherIdleMode)
    .smartCurrentLimit(LauncherConstants.kLauncher1CurrentLimit);
      
    launcher2Config
    .inverted(true)
    .idleMode(LauncherConstants.kLauncherIdleMode)
    .smartCurrentLimit(LauncherConstants.kLauncher2CurrentLimit);
      

    m_launcher1.configure(launcher1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_launcher2.configure(launcher2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
  }
  
  /**
   * Method to lift the climb using joystick info.
   *
   */
  
  public void launchTop(double xSpeed) {
    m_launcher1.set(xSpeed);
  }

  public void launchBottom(double xSpeed) {
    m_launcher2.set(xSpeed);
  }

  public void launch_stop() {
    double passivePower = -0.00;//to eject balls when not launching
    launchTop(passivePower);
    launchBottom(passivePower);
  }

  public void launchBoth(double xSpeed) {
    launchTop(xSpeed);
    launchBottom(0.6);
  }

  public Command launchCommand(double xSpeed) {
    return this.run(() -> launchBoth(xSpeed)).finallyDo(() -> launch_stop());
  }

  public Command launchTopCommand(double xSpeed) {
    return this.run(() -> launchTop(xSpeed)).finallyDo(() -> launch_stop());
 }

  public Command launchStopCommand() {
    return this.runOnce(() -> launch_stop());
  }
  public Command ejectCommand() {
    return this.run(() -> eject()).finallyDo(() -> launch_stop());
  }

  public void eject() {
    launchTop(-1);
    launchBottom(-1);
  }

  public void periodic(){

  }
}
