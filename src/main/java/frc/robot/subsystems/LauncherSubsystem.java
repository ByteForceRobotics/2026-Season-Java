// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import static frc.robot.Constants.LauncherConstants.*;
import edu.wpi.first.units.Units;

public class LauncherSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  SparkFlex m_launcherTopLeft; //top flywheel
  SparkFlex m_launcherTopRight; //bottom 
  SparkMax m_launcherBottomTop; //top flywheel
  SparkMax m_launcherBottomBottom; //bottom 
  double launchPower;

  private final SysIdRoutine leftFlywheelRoutine;
  private final SysIdRoutine rightFlywheelRoutine;

    // Variables below are used exclusively for SysID routine logging.
  private final MutVoltage appliedVoltsRoutine = Units.Volts.mutable(0);
  private final MutAngle angleRoutine = Units.Rotations.mutable(0);
  private final MutAngularVelocity angularVelocityRoutine = Units.RPM.mutable(0);

  public LauncherSubsystem(){
    
    leftFlywheelRoutine = new SysIdRoutine(
		  new SysIdRoutine.Config(),
		  new SysIdRoutine.Mechanism(
			voltage -> {
				m_launcherTopLeft.setVoltage(voltage.in(Units.Volts));
			},
			log -> {
				log.motor("left-shooter-motor")
				  .voltage(appliedVoltsRoutine.mut_replace(Units.Volts.mutable(m_launcherTopLeft.getAppliedOutput()*m_launcherTopLeft.getBusVoltage())))
				  .angularPosition(angleRoutine.mut_replace(Units.Rotations.mutable(MathUtil.clamp(m_launcherTopLeft.getEncoder().getPosition(),-1,1))))
				  .angularVelocity(angularVelocityRoutine.mut_replace(Units.RPM.mutable(m_launcherTopLeft.getEncoder().getVelocity())));
			},
			this
		  )
		);
		rightFlywheelRoutine = new SysIdRoutine(
		  new SysIdRoutine.Config(),
		  new SysIdRoutine.Mechanism(
			voltage -> {
				m_launcherTopRight.setVoltage(voltage.in(Units.Volts));
			},
			log -> {
				log.motor("right-shooter-motor")
				  .voltage(appliedVoltsRoutine.mut_replace(Units.Volts.mutable(m_launcherTopRight.getAppliedOutput()*m_launcherTopLeft.getBusVoltage())))
				  .angularPosition(angleRoutine.mut_replace(Units.Rotations.mutable(MathUtil.clamp(m_launcherTopRight.getEncoder().getPosition(),-1,1))))
				  .angularVelocity(angularVelocityRoutine.mut_replace(Units.RPM.mutable(m_launcherTopRight.getEncoder().getVelocity())));
			},
			this
		  )
		);
    
    m_launcherTopLeft = new SparkFlex(kLauncherTopLeftCanId, MotorType.kBrushless);//top
    m_launcherTopRight = new SparkFlex(kLauncherTopRightCanId, MotorType.kBrushless);//bottom
    m_launcherBottomTop = new SparkMax(kLauncherBottomTopCanId, MotorType.kBrushless);//top
    m_launcherBottomBottom = new SparkMax(kLauncherBottomBottomCanId, MotorType.kBrushless);//bottom
    launchPower = 0;
    SparkMaxConfig topRightLauncherConfig = new SparkMaxConfig();
    SparkMaxConfig topLeftLauncherConfig = new SparkMaxConfig();
    SparkMaxConfig middleLauncherConfig = new SparkMaxConfig();
    SparkMaxConfig bottomLauncherConfig = new SparkMaxConfig();
    
    // Velocity control PID constants for RPM control
    // NEO motors max out around 5700 RPM
    

    SparkFlexConfig TopConfig = new SparkFlexConfig();
    TopConfig
        .closedLoop.pid(kTopP, kTopI, kTopD).outputRange(0, 1)
        .allowedClosedLoopError(kTopTolerance, ClosedLoopSlot.kSlot0)
        .feedForward.sva(0.01,1/525,0.001);

    SparkMaxConfig MiddleConfig = new SparkMaxConfig();
    MiddleConfig
        .closedLoop.pid(kBottomTopP, kBottomTopI, kBottomTopD).outputRange(0, 1)
        .allowedClosedLoopError(kBottomTolerance, ClosedLoopSlot.kSlot0)
        .feedForward.sva(0.1, 1/545 , 0.00003);

    SparkMaxConfig BottomConfig = new SparkMaxConfig();
    BottomConfig
        .closedLoop.pid(kBottomBottomP, kBottomBottomI, kBottomBottomD).outputRange(0, 1)
        .allowedClosedLoopError(kBottomTolerance, ClosedLoopSlot.kSlot0)
        .feedForward.sva(0.1, 1/545 , 0.00003);
      
    topRightLauncherConfig
      .inverted(true)
      .idleMode(kLauncherTopIdleMode)
      .smartCurrentLimit(kLauncher1CurrentLimit)
      .apply(TopConfig);

    topLeftLauncherConfig
      .inverted(false)
      .idleMode(kLauncherTopIdleMode)
      .smartCurrentLimit(kLauncher1CurrentLimit)
      .apply(TopConfig);

    middleLauncherConfig
      .inverted(false)
      .idleMode(kLauncherTopIdleMode)
      .smartCurrentLimit(kLauncher1CurrentLimit)
      .apply(MiddleConfig);

    bottomLauncherConfig
      .inverted(false)
      .idleMode(kLauncherBottomIdleMode)
      .smartCurrentLimit(kLauncher1CurrentLimit)
      .apply(BottomConfig);
    

    
    m_launcherTopLeft.configure(topLeftLauncherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_launcherTopRight.configure(topRightLauncherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_launcherBottomTop.configure(middleLauncherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_launcherBottomBottom.configure(bottomLauncherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    SparkClosedLoopController topController =  m_launcherTopLeft.getClosedLoopController();
  }
  
   // SysId getters
  public Command leftShooterQuasiForward() {return leftFlywheelRoutine.quasistatic(SysIdRoutine.Direction.kForward);}

	public Command rightShooterQuasiForward() {return rightFlywheelRoutine.quasistatic(SysIdRoutine.Direction.kForward);}

	public Command leftShooterQuasiReverse() {return leftFlywheelRoutine.quasistatic(SysIdRoutine.Direction.kReverse);}

	public Command rightShooterQuasiReverse() {return rightFlywheelRoutine.quasistatic(SysIdRoutine.Direction.kReverse);}

	public Command leftShooterDynamicForward() {return leftFlywheelRoutine.dynamic(SysIdRoutine.Direction.kForward);}

	public Command rightShooterDynamicForward() {return rightFlywheelRoutine.dynamic(SysIdRoutine.Direction.kForward);}

	public Command leftShooterDynamicReverse() {return leftFlywheelRoutine.dynamic(SysIdRoutine.Direction.kReverse);}

	public Command rightShooterDynamicReverse() {return rightFlywheelRoutine.dynamic(SysIdRoutine.Direction.kReverse);}

  public Command RightsysIdQuasistatic(SysIdRoutine.Direction direction) {
    return rightFlywheelRoutine.quasistatic(direction);
  }

  public Command RightsysIdDynamic(SysIdRoutine.Direction direction) {
    return rightFlywheelRoutine.dynamic(direction);
  }
  public Command LeftsysIdQuasistatic(SysIdRoutine.Direction direction) {
    return leftFlywheelRoutine.quasistatic(direction);
  }

  public Command LeftsysIdDynamic(SysIdRoutine.Direction direction) {
    return leftFlywheelRoutine.dynamic(direction);
  }

  
  public void launchTop(double xSpeed) {
    m_launcherTopLeft.set(xSpeed);
    m_launcherTopRight.set(xSpeed);
  }

  public void launchTopRightVolts(double xSpeed) {
    m_launcherTopRight.setVoltage(xSpeed);
  }

  public void launchTopLeftVolts(double xSpeed) {
    m_launcherTopLeft.setVoltage(xSpeed);
  }

  public void launchBottom(double xSpeed) {
    m_launcherBottomTop.set(xSpeed);
    m_launcherBottomBottom.set(xSpeed);
  }

  public void launch_stop() {
    double passivePower = -0.00;//to eject balls when not launching
    launchTop(passivePower);
    launchBottom(passivePower);
  }

  public void launchBoth(double xSpeed) {
    launchTop(xSpeed);
    System.out.println(xSpeed);
    launchBottom(xSpeed/4);
  }
  public void updateLaunchPower(double power){
    launchPower = power;
  }
  public void updateLaunchTopRPM(double topRPM){
    launchTopRPM(topRPM);
  }
  public void updateLaunchBottomRPM(double bottomRPM){
    launchBottomRPM(bottomRPM);
  }

  /**
   * Control launcher top by target RPM using closed-loop velocity control.
   * @param targetRPM Target rotations per minute
   */
  public void launchTopRPM(double targetRPM) {
    m_launcherTopLeft.getClosedLoopController().setSetpoint(targetRPM, ControlType.kVelocity);
    m_launcherTopRight.getClosedLoopController().setSetpoint(targetRPM, ControlType.kVelocity);
  }

  /**
   * Control launcher bottom by target RPM using closed-loop velocity control.
   * @param targetRPM Target rotations per minute
   */
  public void launchBottomRPM(double targetRPM) {
    m_launcherBottomTop.getClosedLoopController().setSetpoint(targetRPM, ControlType.kVelocity);
    m_launcherBottomBottom.getClosedLoopController().setSetpoint(targetRPM, ControlType.kVelocity);
  }

  /**
   * Control both launchers by target RPM.
   * @param topRPM Target RPM for top launcher
   * @param bottomRPM Target RPM for bottom launcher
   */
  public void launchBothRPM(double topRPM, double bottomRPM) {
    launchTopRPM(topRPM);
    launchBottomRPM(bottomRPM);
  }

  /**
   * Get current average RPM of top launcher.
   * @return RPM of top launcher
   */
  public double getTopRPM() {
    return (m_launcherTopLeft.getEncoder().getVelocity() + m_launcherTopRight.getEncoder().getVelocity()) / 2.0;
  }

  /**
   * Get current average RPM of bottom launcher.
   * @return RPM of bottom launcher
   */
  public double getBottomRPM() {
    return (m_launcherBottomTop.getEncoder().getVelocity() + m_launcherBottomBottom.getEncoder().getVelocity()) / 2.0;
  }

  public Command launchCommand(double xSpeed) {
    return this.run(() -> launchBoth(xSpeed)).finallyDo(() -> launch_stop());
  }

  public Command launchTopCommand(double xSpeed) {
    return this.run(() -> launchTop(xSpeed)).finallyDo(() -> launch_stop());
   }
  public Command launchBottomCommand(double xSpeed){
    return this.run(() -> launchBottom(xSpeed)).finallyDo(() -> launch_stop());
  }



  //method  overloading to have adjustable power, and adjustablle auto power
  public Command launchCommand() {
    return this.run(() -> launchBoth(launchPower)).finallyDo(() -> launch_stop());
  }
  public Command launchTopCommand() {
    return this.run(() -> launchTop(launchPower)).finallyDo(() -> launch_stop());
   }
  public Command launchBottomCommand(){
    return this.run(() -> launchBottom(launchPower)).finallyDo(() -> launch_stop());
  }
  // ===== RPM-based command methods =====

  /**
   * Create a command to launch with specific RPM targets.
   * @param topRPM Target RPM for top launcher
   * @param bottomRPM Target RPM for bottom launcher
   * @return Command that runs launchers at target RPM
   */
  public Command launchCommandRPM(double topRPM, double bottomRPM) {
    return this.run(() -> launchBothRPM(topRPM, bottomRPM)).finallyDo(() -> launch_stop());
  }

  /**
   * Create a command to launch top motor at specific RPM.
   * @param topRPM Target RPM for top launcher
   * @return Command that runs top launcher at target RPM
   */
  public Command launchTopCommandRPM(double topRPM) {
    return this.run(() -> launchTopRPM(topRPM)).finallyDo(() -> launch_stop());
  }

  /**
   * Create a command to launch bottom motor at specific RPM.
   * @param bottomRPM Target RPM for bottom launcher
   * @return Command that runs bottom launcher at target RPM
   */
  public Command launchBottomCommandRPM(double bottomRPM) {
    return this.run(() -> launchBottomRPM(bottomRPM)).finallyDo(() -> launch_stop());
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

  /**
   * Get current RPM of top left motor.
   * @return RPM of top left motor
   */
  public double getTopLeftRPM() {
    return m_launcherTopLeft.getEncoder().getVelocity();
  }

  /**
   * Get current RPM of top right motor.
   * @return RPM of top right motor
   */
  public double getTopRightRPM() {
    return m_launcherTopRight.getEncoder().getVelocity();
  }

  /**
   * Get current RPM of bottom top motor.
   * @return RPM of bottom top motor
   */
  public double getBottomTopRPM() {
    return m_launcherBottomTop.getEncoder().getVelocity();
  }

  /**
   * Get current RPM of bottom bottom motor.
   * @return RPM of bottom bottom motor
   */
  public double getBottomBottomRPM() {
    return m_launcherBottomBottom.getEncoder().getVelocity();
  }

  /**
   * Control top left motor by target RPM.
   * @param targetRPM Target RPM for top left motor
   */
  public void launchTopLeftRPM(double targetRPM) {
    m_launcherTopLeft.getClosedLoopController().setSetpoint(targetRPM, ControlType.kVelocity);
  }

  /**
   * Control top right motor by target RPM.
   * @param targetRPM Target RPM for top right motor
   */
  public void launchTopRightRPM(double targetRPM) {
    m_launcherTopRight.getClosedLoopController().setSetpoint(targetRPM, ControlType.kVelocity);
  }

  /**
   * Control bottom top motor by target RPM.
   * @param targetRPM Target RPM for bottom top motor
   */
  public void launchBottomTopRPM(double targetRPM) {
    m_launcherBottomTop.getClosedLoopController().setSetpoint(targetRPM, ControlType.kVelocity);
  }

  /**
   * Control bottom bottom motor by target RPM.
   * @param targetRPM Target RPM for bottom bottom motor
   */
  public void launchBottomBottomRPM(double targetRPM) {
    m_launcherBottomBottom.getClosedLoopController().setSetpoint(targetRPM, ControlType.kVelocity);
  }

  public void periodic(){
    // Publish current RPM to SmartDashboard for monitoring
  }

  public double getTopLeftOutput() {
    return m_launcherTopLeft.getAppliedOutput();
  }
  public double getTopRightOutput() {
    return m_launcherTopRight.getAppliedOutput();
  }
  public double getMiddleOutput() {
    return m_launcherBottomTop.getAppliedOutput();
  }
  public double getBottomOutput() {
    return m_launcherBottomBottom.getAppliedOutput();
  }
}
