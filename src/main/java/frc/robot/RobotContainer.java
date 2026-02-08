// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

//import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LauncherConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer {
  // The robot's subsystems
  private final SendableChooser<Command> autoChooser;
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ClimbSubsystem m_climber = new ClimbSubsystem();
  private final VisionSubsystem m_vision = new VisionSubsystem();
  private final LauncherSubsystem m_launcher = new LauncherSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_climberController = new XboxController(OIConstants.kClimbControllerPort);
  //CommandXboxController m_commanddriverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  //https://www.padcrafter.com/?templates=Controller+Scheme+1&leftTrigger=&rightTrigger=slow+down%2C+to+be+added&rightBumper=&leftBumper=&leftStickClick=change+scale&leftStick=movement%2Fdriving&rightStick=rotation&xButton=&aButton=spin+intake+motors&yButton=launch+balls+&startButton=&bButton=reset+heading&dpadUp=climb+up&dpadDown=climb+reverse&dpadRight=deploy+lifter&dpadLeft=retract+lifter
  // this is the link to the controller keybinds
  


  boolean fieldRelative = true;
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  //simulation
                                                                                                          
  //photon cam
  // java -jar "C:\FRC Code\PhotonVision\photonvision-v2025.3.1-winx64.jar"    
  
  
  // differnet speed modes
  double speedScaleHigh = 1.0;
  double speedScaleLow= 0.33;
  double speedScale = speedScaleHigh;

  SequentialCommandGroup climblvl1= new RunCommand(
        () -> m_climber.climb(ClimbConstants.kClimbSpeed),// note: i dont know if this will stop climbing the moment the time runs out
        m_climber).withTimeout(ClimbConstants.kLevelOneTime).andThen(new InstantCommand(
                () -> m_climber.pull_stop(),
                m_climber));
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    
    autoChooser = AutoBuilder.buildAutoChooser();
    
    // Configure default commands
    m_robotDrive.setDefaultCommand(
      // The left stick controls translation of the robot.
      // Turning is controlled by the X axis of the right stick.
      // fieldRelative
      new RunCommand(
        () -> m_robotDrive.drive(
            -MathUtil.applyDeadband(m_driverController.getLeftY()*speedScale, OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband(m_driverController.getLeftX()*speedScale, OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband(m_driverController.getRightX()*speedScale, OIConstants.kDriveDeadband),
            fieldRelative),
        m_robotDrive));
    

    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
  
  public void periodic() {
    
  }

  public static Trigger triggerButton (XboxController controller, XboxController.Axis axis) {
    return new Trigger(() -> controller.getRawAxis(axis.value) >= Constants.OIConstants.kDriveDeadband);
  }
  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}),
   * and then calling passing it to a {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    /* 
    new JoystickButton(m_climberController, Button.kStart.value)
    .onTrue(new RunCommand(
      () -> m_reef.moveCoral(0.165),
      m_climber).withTimeout(1.0).andThen(new InstantCommand(
        () -> m_reef.moveCoral_stop(),
        m_climber)));
    */
    
    /*
    new JoystickButton(m_climberController, Button.kBack.value)//toggles limits on and off
        .onTrue(new InstantCommand(
            () -> m_climber.toggleElevatorFixMode(),m_climber));
    */
    /* 
    old bumper keybinds
    new JoystickButton(m_climberController, Button.kLeftBumper.value)
        .whileTrue(new RunCommand(
            () -> m_reef.moveCoral(-0.165),m_reef))//takes coral back in
            .onFalse(new InstantCommand(
                () -> m_reef.moveCoral_stop(),m_reef));

    new JoystickButton(m_climberController, Button.kRightBumper.value)
        .whileTrue(new RunCommand(
            () -> m_reef.moveCoral(0.165),m_reef))//dispenses coral
            .onFalse(new InstantCommand(
                () -> m_reef.moveCoral_stop(),m_reef));
    */

    new JoystickButton(m_driverController, Button.kLeftStick.value)
        .onTrue(new InstantCommand(
            () -> changeScale()));//make this trigger




    new POVButton(m_driverController, 0)
        .whileTrue(new RunCommand(
            () -> m_climber.climb(ClimbConstants.kClimbSpeed),
            m_climber)).onFalse(new InstantCommand(
                () -> m_climber.pull_stop(),
                m_climber));
    new POVButton(m_driverController, 180)
        .whileTrue(new RunCommand(
            () -> m_climber.climb(-ClimbConstants.kClimbSpeed),
            m_climber)).onFalse(new InstantCommand(
                () -> m_climber.pull_stop(),
                m_climber));
    
    new POVButton(m_driverController, 90)
        .whileTrue(new RunCommand(
            () -> m_intake.lift(IntakeConstants.kLiftDefaultSpeed),
            m_intake)).onFalse(new InstantCommand(
                () -> m_intake.lift_stop(),
                m_intake));
    new POVButton(m_driverController, 270)
        .whileTrue(new RunCommand(
            () -> m_intake.lift(-IntakeConstants.kLiftDefaultSpeed),
            m_intake)).onFalse(new InstantCommand(
                () -> m_intake.lift_stop(),
                m_intake));
                
    /*
    trigger buttons, might be useful
    triggerButton(m_climberController,Axis.kLeftTrigger).whileTrue(new RunCommand(
        () -> m_climber.climb(m_climberController.getRawAxis(Axis.kLeftTrigger.value)),
        m_climber)).onFalse(new InstantCommand(
            () -> m_climber.climb_stop(),
            m_climber));

    triggerButton(m_climberController,Axis.kRightTrigger).whileTrue(new RunCommand(
        () -> m_climber.climb(-m_climberController.getRawAxis(Axis.kRightTrigger.value)),
        m_climber)).onFalse(new InstantCommand(
            () -> m_climber.climb_stop(),
            m_climber));
    */

    
    new JoystickButton(m_driverController, Button.kY.value)
        .whileTrue(new RunCommand(
            () -> m_launcher.launch(LauncherConstants.kLauncherDefaultSpeed),
            m_launcher)).onFalse(new InstantCommand(
            () -> m_launcher.launch_stop(),
            m_launcher));   

    
    new JoystickButton(m_driverController, Button.kB.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.zeroHeading(),
            m_robotDrive));     
            
    
     new JoystickButton(m_driverController, Button.kA.value)
        .whileTrue(new RunCommand(
            () -> m_intake.intake(IntakeConstants.kIntakeDefaultSpeed),
            m_intake)).onFalse(new InstantCommand(
            () -> m_intake.intake_stop(),
            m_intake));   
    
    //add POV buttons(d-pad) for strafing
    /* 
    new POVButton(m_driverController, 0)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.bumper(0.5, 0),m_robotDrive));
    new POVButton(m_driverController, 90)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.bumper(0.5, 90),m_robotDrive));
    new POVButton(m_driverController, 180)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.bumper(0.5, 180),m_robotDrive));
    new POVButton(m_driverController, 270)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.bumper(0.5, 270),m_robotDrive));
     */
  }

  public void changeScale(){
    if(speedScale == speedScaleHigh){
      speedScale = speedScaleLow;
    } else {
      speedScale = speedScaleHigh;
    }
  }
}
