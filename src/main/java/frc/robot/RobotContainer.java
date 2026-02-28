// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

//import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import java.util.Map;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AgitatorConstants;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LauncherConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.AgitatorSubsystem;
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
  private final VisionSubsystem m_vision = new VisionSubsystem((pose, timestamp, stdDevs, forceReset) -> {
    if (forceReset) {
      m_robotDrive.resetOdometry(pose);
    } else {
      m_robotDrive.addVisionMeasurement(pose, timestamp, stdDevs);
    }
  }, m_robotDrive::getPose);
  private final ClimbSubsystem m_climber = new ClimbSubsystem();
  //private final VisionSubsystem m_vision = new VisionSubsystem();
  private final LauncherSubsystem m_launcher = new LauncherSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final AgitatorSubsystem m_agitator = new AgitatorSubsystem();
  private double slowdownMultiplier = 1;
  double launcherSpeed = LauncherConstants.kLauncherDefaultSpeed;

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
        () -> m_climber.climb(ClimbConstants.kClimbSpeed),
        m_climber).withTimeout(ClimbConstants.kLevelOneTime).andThen(new InstantCommand(
                () -> m_climber.pull_stop(),
                m_climber));
  SequentialCommandGroup autoShoot = new RunCommand(()-> m_launcher.launch(.8),m_launcher).withTimeout(1).andThen(new InstantCommand(() -> m_launcher.launch_stop(),m_launcher));
  public RobotContainer() {
    NamedCommands.registerCommand("LevelOneClimb", climblvl1);
    NamedCommands.registerCommand("ShootOneSec", autoShoot);
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
            -MathUtil.applyDeadband(m_driverController.getLeftY()*speedScale*slowdownMultiplier, OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband(m_driverController.getLeftX()*speedScale*slowdownMultiplier, OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband(m_driverController.getRightX()*speedScale*slowdownMultiplier, OIConstants.kDriveDeadband),
            fieldRelative),
        m_robotDrive));

      
    

    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");
    
    SmartDashboard.putData("Auto Chooser", autoChooser);
    // Add a SmartDashboard tunable for launcher speed but don't overwrite any existing value
    if (!SmartDashboard.containsKey("Launcher/Speed")) {
      SmartDashboard.putNumber("Launcher/Speed", launcherSpeed);
    }
    // Add a Shuffleboard slider bound to the same NetworkTable key so users can edit it visually
    var launcherTab = Shuffleboard.getTab("Launcher");
    launcherTab.add("Launcher/Speed", SmartDashboard.getNumber("Launcher/Speed", launcherSpeed))
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0, "max", 1));
    launcherTab.addNumber("Speed Current", () -> launcherSpeed);
  }
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
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
        .onTrue(new InstantCommand(() -> changeScale()));//make this trigger

    new POVButton(m_driverController, 0)
        .whileTrue(new InstantCommand(
            () -> m_climber.climb(ClimbConstants.kClimbSpeed),
            m_climber)).onFalse(new InstantCommand(
                () -> m_climber.pull_stop(),
                m_climber));
    
    new POVButton(m_driverController, 180)
        .whileTrue(new InstantCommand(
            () -> m_climber.climb(-ClimbConstants.kClimbSpeed),
            m_climber)).onFalse(new InstantCommand(
                () -> m_climber.pull_stop(),
                m_climber));
    
    new POVButton(m_driverController, 90)
        .whileTrue(new InstantCommand(
            () -> m_intake.lift(IntakeConstants.kLiftDefaultSpeed),
            m_intake)).onFalse(new InstantCommand(
                () -> m_intake.lift_stop(),
                m_intake));

    new POVButton(m_driverController, 270)
        .whileTrue(new InstantCommand(
            () -> m_intake.lift(-IntakeConstants.kLiftDefaultSpeed),
            m_intake)).onFalse(new InstantCommand(
                () -> m_intake.lift_stop(),
                m_intake));

    triggerButton(m_driverController,Axis.kLeftTrigger)
      .whileTrue(new RunCommand(() -> slowdown(m_driverController.getRawAxis(Axis.kLeftTrigger.value))))
      .onFalse(new InstantCommand(() -> slowdown_stop()));
    /* 
    triggerButton(m_driverController,Axis.kRightTrigger)
      .whileTrue(new RunCommand(() -> m_launcher.launch(m_driverController.getRawAxis(Axis.kRightTrigger.value)), m_launcher))
      .onFalse(new InstantCommand(() -> m_launcher.launch_stop(), m_launcher));
    */
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
    ///*
    new JoystickButton(m_driverController, Button.kRightBumper.value)
        .whileTrue(
            new InstantCommand(() -> m_launcher.launchTop(launcherSpeed), m_launcher).withTimeout(2)
                .beforeStarting(() -> m_launcher.launch(launcherSpeed), m_launcher)
                .alongWith(new InstantCommand(() -> m_agitator.agitate(AgitatorConstants.kAgitatorDefaultSpeed), m_agitator)))
        
        .onFalse(new InstantCommand(() -> { m_launcher.launch_stop(); m_agitator.agitate_stop(AgitatorConstants.kAgitatorDefaultSpeed); }, m_launcher, m_agitator));
    //*/
    /* 
    SequentialCommandGroup launchSuperCool = new RunCommand(
        () -> m_launcher.launch(LauncherConstants.kLauncherDefaultSpeed),
        m_launcher).alongWith(new RunCommand(
        () -> m_agitator.agitate(AgitatorConstants.kAgitatorDefaultSpeed),
        m_agitator)).beforeStarting(new RunCommand(
        () -> m_launcher.launchTop(LauncherConstants.kLauncherDefaultSpeed),
        m_launcher).withTimeout(0.5)).andThen(new InstantCommand(
        () -> m_launcher.launch_stop(),
        m_launcher).alongWith(new RunCommand(
        () -> m_agitator.agitate_stop(),
        m_agitator)));
      */
    ///*
    new JoystickButton(m_driverController, Button.kRightStick.value)
        .whileTrue(new InstantCommand(() -> m_launcher.eject(), m_launcher))
        .onFalse(new InstantCommand(() -> m_launcher.launch_stop(), m_launcher));

    new JoystickButton(m_driverController, Button.kB.value)
        .whileTrue(new InstantCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive));

    new JoystickButton(m_driverController, Button.kLeftBumper.value)// make the intake toggleable/ and or left bumper
        .whileTrue(new InstantCommand(() -> m_intake.intake(Constants.IntakeConstants.kIntakeDefaultSpeed), m_intake))
        .onFalse(new InstantCommand(() -> m_intake.intake_stop(), m_intake));
      
    new JoystickButton(m_driverController, Button.kX.value)
      .onTrue(new InstantCommand(() -> m_agitator.agitate_toggle(), m_agitator));
    
    new JoystickButton(m_driverController, Button.kStart.value)
      .whileTrue(new RunCommand(() -> m_agitator.agitate(AgitatorConstants.kAgitatorDefaultSpeed), m_agitator))
      .onFalse(new InstantCommand(() -> m_agitator.agitate_stop(AgitatorConstants.kAgitatorDefaultSpeed), m_agitator));
    
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

  public void slowdown(double val){
    slowdownMultiplier = 1-val;
    SmartDashboard.putNumber("slowdown multiplier", slowdownMultiplier);
  }
  public void slowdown_stop(){
    slowdownMultiplier = 1;
    SmartDashboard.putNumber("slowdown multiplier", slowdownMultiplier);
  }
  public void periodic() {
    // Read the launcher speed from the dashboard so it can be tuned at runtime
    launcherSpeed = SmartDashboard.getNumber("Launcher/Speed", launcherSpeed);
    SmartDashboard.putNumber("Launcher/Speed_Current", launcherSpeed);
  }
}
