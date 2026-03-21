// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AgitatorConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LauncherConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.GoToPositionLifterCommand;
import frc.robot.commands.LauncherPIDCommand;
import frc.robot.commands.TurnToTagCommand;
import frc.robot.subsystems.AgitatorSubsystem;
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
  private final VisionSubsystem m_vision = new VisionSubsystem();
  //private final ClimbSubsystem m_climber = new ClimbSubsystem();
  //private final VisionSubsystem m_vision = new VisionSubsystem();
  private final LauncherSubsystem m_launcher = new LauncherSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final AgitatorSubsystem m_agitator = new AgitatorSubsystem();
  private double slowdownMultiplier = 1;
  double launcherSpeed = LauncherConstants.kLauncherSpeed;

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
  double launchPower = LauncherConstants.kLauncherSpeed;
  

  // SequentialCommandGroup climblvl1= new RunCommand(
  //       () -> m_climber.climb(ClimbConstants.kClimbSpeed), m_climber)
  //       .withTimeout(ClimbConstants.kLevelOneTime)
  //       .andThen(new InstantCommand(() -> m_climber.pull_stop(), m_climber));
  
  double autoShootLaunchSpeed = 0.65;
  SequentialCommandGroup autoShoot =new LauncherPIDCommand(m_launcher,m_vision,10,10).withTimeout(5)
            .alongWith(delayCommand(LauncherConstants.kBottomLauncherDelay).andThen(m_agitator.agitateCommand(AgitatorConstants.kAgitatorDefaultSpeed))).andThen(m_agitator.agitateStopCommand(0));
  //ParallelRaceGroup driveBackwards1Seconds = m_robotDrive.driveCommand(0,-.5,0,true).withTimeout(2);
  //SequentialCommandGroup hopeCore  = driveBackwards1Seconds.andThen(turnToTagCommand().withTimeout(2)).andThen(autoShoot);
  
  public RobotContainer() {
    //NamedCommands.registerCommand("LevelOneClimb", climblvl1);
    NamedCommands.registerCommand("ShootAllBalls", autoShoot);
    //NamedCommands.registerCommand("DriveBackwards1Seconds", driveBackwards1Seconds);
    NamedCommands.registerCommand("TurnToTagCommand", turnToTagCommand());
    NamedCommands.registerCommand("IntakeToggleCommand", m_intake.intakeToggleCommand());
    NamedCommands.registerCommand("LifterLower", new GoToPositionLifterCommand(m_intake, IntakeConstants.kLifterMaxLower));
    NamedCommands.registerCommand("LifterLift", new GoToPositionLifterCommand(m_intake, IntakeConstants.kLifterMaxLift));
    
    
    // Add RPM sliders for manual control (range 0-5700 RPM for NEO motors)
    SmartDashboard.putNumber("Launcher/GoalTopRPM",100);
    SmartDashboard.putNumber("Launcher/GoalBottomRPM",100);
    
    // Top Launcher PID constants
    SmartDashboard.putNumber("Launcher/PID/Top/kP", LauncherConstants.kTopP);
    SmartDashboard.putNumber("Launcher/PID/Top/kI", LauncherConstants.kTopI);
    SmartDashboard.putNumber("Launcher/PID/Top/kD", LauncherConstants.kTopD);
    SmartDashboard.putNumber("Launcher/PID/Top/Tolerance", LauncherConstants.kTopTolerance);
    
    // Bottom-Top Launcher PID constants
    SmartDashboard.putNumber("Launcher/PID/BottomTop/kP", LauncherConstants.kBottomTopP);
    SmartDashboard.putNumber("Launcher/PID/BottomTop/kI", LauncherConstants.kBottomTopI);
    SmartDashboard.putNumber("Launcher/PID/BottomTop/kD", LauncherConstants.kBottomTopD);
    SmartDashboard.putNumber("Launcher/PID/BottomTop/Tolerance", LauncherConstants.kBottomTolerance);
    
    // Bottom-Bottom Launcher PID constants
    SmartDashboard.putNumber("Launcher/PID/BottomBottom/kP", LauncherConstants.kBottomBottomP);
    SmartDashboard.putNumber("Launcher/PID/BottomBottom/kI", LauncherConstants.kBottomBottomI);
    SmartDashboard.putNumber("Launcher/PID/BottomBottom/kD", LauncherConstants.kBottomBottomD);
    SmartDashboard.putNumber("Launcher/PID/BottomBottom/Tolerance", LauncherConstants.kBottomTolerance);
    

    SmartDashboard.putNumber("Lifter/PID/kP", IntakeConstants.kP);
    SmartDashboard.putNumber("Lifter/PID/kI", IntakeConstants.kI);
    SmartDashboard.putNumber("Lifter/PID/kD", IntakeConstants.kD);
    SmartDashboard.putNumber("Lifter/PID/kTolerance", IntakeConstants.kTolerance);
    // Bottom launcher delay to avoid stutter (in seconds)
    SmartDashboard.putNumber("Launcher/BottomDelay", LauncherConstants.kBottomLauncherDelay);
    
    // Configure the button bindings
    configureButtonBindings();
    
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    
    // Setup Shuffleboard launcher control tab
    
    // Configure default commands
    m_robotDrive.setDefaultCommand(
      // The left stick controls translation of the robot.
      // Turning is controlled by the X axis of the right stick.
      // fieldRelative
      new RunCommand(
      () -> {
        double rawLeftY = m_driverController.getLeftY() * speedScale * slowdownMultiplier;
        double rawLeftX = m_driverController.getLeftX() * speedScale * slowdownMultiplier;
        double rawRightX = m_driverController.getRightX() * speedScale * slowdownMultiplier;

        double dbLeftY = MathUtil.applyDeadband(rawLeftY, OIConstants.kDriveDeadband);
        double dbLeftX = MathUtil.applyDeadband(rawLeftX, OIConstants.kDriveDeadband);
        double dbRightX = MathUtil.applyDeadband(rawRightX, OIConstants.kDriveDeadband);

        double sqLeftY = Math.copySign(dbLeftY * dbLeftY, dbLeftY);
        double sqLeftX = Math.copySign(dbLeftX * dbLeftX, dbLeftX);
        double sqRightX = Math.copySign(dbRightX * dbRightX, dbRightX);

        m_robotDrive.drive(-sqLeftY, -sqLeftX, -sqRightX, fieldRelative);
      },
      m_robotDrive));

    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");
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

    // new POVButton(m_driverController, 0)
    //     .whileTrue(new InstantCommand(
    //         () -> m_climber.climb(ClimbConstants.kClimbSpeed),
    //         m_climber)).onFalse(new InstantCommand(
    //             () -> m_climber.pull_stop(),
    //             m_climber));
    
    // new POVButton(m_driverController, 180)
    //     .whileTrue(new InstantCommand(
    //         () -> m_climber.climb(-ClimbConstants.kClimbSpeed),
    //         m_climber)).onFalse(new InstantCommand(
    //             () -> m_climber.pull_stop(),
    //             m_climber));
    
    new POVButton(m_driverController, 90)
        .onTrue(new GoToPositionLifterCommand(m_intake, IntakeConstants.kLifterMaxLower));

    new POVButton(m_driverController, 270)
        .onTrue(new GoToPositionLifterCommand(m_intake, IntakeConstants.kLifterMaxLift));

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
    new JoystickButton(m_driverController, Button.kBack.value)
      .onTrue(autoShoot);
    
    new JoystickButton(m_driverController, Button.kRightBumper.value)
        .whileTrue(new LauncherPIDCommand(m_launcher,m_vision,10,10)
            .alongWith(delayCommand(LauncherConstants.kBottomLauncherDelay).andThen(m_agitator.agitateCommand(AgitatorConstants.kAgitatorDefaultSpeed))))
        .onFalse(m_agitator.agitateStopCommand(0)); 

    new JoystickButton(m_driverController, Button.kRightStick.value)
        .whileTrue(m_launcher.ejectCommand().alongWith(m_agitator.agitateCommand(-AgitatorConstants.kAgitatorDefaultSpeed)))
        .onFalse(m_launcher.launchStopCommand());

    new JoystickButton(m_driverController, Button.kB.value)
        .onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive));
        
    new JoystickButton(m_driverController, Button.kA.value)
        .whileTrue(new TurnToTagCommand(m_robotDrive, m_vision));

    new JoystickButton(m_driverController, Button.kLeftBumper.value)// make the intake toggleable/ and or left bumper
        .onTrue(m_intake.intakeToggleCommand());
      
    new JoystickButton(m_driverController, Button.kX.value)
        .onTrue(m_agitator.agitateToggleCommand());
    
    new JoystickButton(m_driverController, Button.kStart.value)
        .whileTrue(m_agitator.agitateCommand(AgitatorConstants.kAgitatorDefaultSpeed))
        .onFalse(m_agitator.agitateStopCommand(0));
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
  }

  public void slowdown_stop(){
    slowdownMultiplier = 1;
  }
  
  public Command turnToTagCommand(){
    return new TurnToTagCommand(m_robotDrive, m_vision);
  }

  public void periodic() {
    // Initialize launch power on dashboard if not already present
    
    
    // Read launch power from dashboard so it can be tuned at runtime
    //System.out.println("launchPower1 " + launchPower);

    // Read RPM values from sliders for manual control
    
    // If either slider is non-zero, apply those RPM values

    //System.out.println("launchPower2 " + launchPower);
    SmartDashboard.putNumber("slowdown multiplier", slowdownMultiplier);
    double distance = m_vision.getDistance();
    SmartDashboard.putNumber("CamDistance",distance);
    double horizontalDistance = distance*Math.cos(Math.toRadians(m_vision.getPitch()));
    SmartDashboard.putNumber("CamCalcHorizDistance", horizontalDistance);
  }

  public double calcRPM(double horizontalDistance) {
    return distanceFunc(horizontalDistance);
  }
  public double distanceFunc(double horizontalDistance){
    double func = horizontalDistance;
    return func;
  }
  private Command delayCommand(double seconds) {
    return new RunCommand(()-> {})
        .withTimeout(seconds);
}
          
}
