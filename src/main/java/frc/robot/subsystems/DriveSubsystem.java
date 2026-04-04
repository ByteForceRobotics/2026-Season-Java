// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase{
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor
  //private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();
  //private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();
  private final AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI);
  PIDController PIDTurnGyro;
  static final double kP = 0.03;
  static final double kI = 0.00;
  static final double kD = 0.00;
  static final double kF = 0.00;

  //simulation
  private final Field2d m_field = new Field2d();
  
  private double m_lastCmdVx = 0.0;
  private double m_lastCmdVy = 0.0;
  private double m_lastCmdOmega = 0.0;

  private final DoubleLogEntry cmdVxLog;
  private final DoubleLogEntry cmdVyLog;
  private final DoubleLogEntry cmdOmegaLog;
  private final DoubleLogEntry measVxLog;
  private final DoubleLogEntry measVyLog;
  private final DoubleLogEntry measOmegaLog;

  private final DoubleLogEntry flDriveSetpointLog;
  private final DoubleLogEntry flDriveMeasuredLog;
  private final DoubleLogEntry flTurnSetpointLog;
  private final DoubleLogEntry flTurnMeasuredLog;

  private final DoubleLogEntry frDriveSetpointLog;
  private final DoubleLogEntry frDriveMeasuredLog;
  private final DoubleLogEntry frTurnSetpointLog;
  private final DoubleLogEntry frTurnMeasuredLog;

  private final DoubleLogEntry rlDriveSetpointLog;
  private final DoubleLogEntry rlDriveMeasuredLog;
  private final DoubleLogEntry rlTurnSetpointLog;
  private final DoubleLogEntry rlTurnMeasuredLog;

  private final DoubleLogEntry rrDriveSetpointLog;
  private final DoubleLogEntry rrDriveMeasuredLog;
  private final DoubleLogEntry rrTurnSetpointLog;
  private final DoubleLogEntry rrTurnMeasuredLog;
  
  // Odometry class for tracking robot pose
  SwerveDrivePoseEstimator m_driveEstimator;

  /** Creates a new DriveSubsystem. */
  
  public DriveSubsystem(){

    var stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
    var visionStdDevs = VecBuilder.fill(1, 1, 1);

    m_driveEstimator = new SwerveDrivePoseEstimator(
      DriveConstants.kDriveKinematics,
      m_gyro.getRotation2d(),
      getModulePositions(),
      new Pose2d(),
      stateStdDevs,
      visionStdDevs);
    
    // Usage reporting for MAXSwerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);
    RobotConfig config;
    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      config = null;
      // Handle exception as needed
      e.printStackTrace();
    }

    PIDTurnGyro = new PIDController(kP, kI, kD);
    PIDTurnGyro.setTolerance(5.0); // tolerance in degrees
    PIDTurnGyro.enableContinuousInput(-180, 180); // for continuous rotation
    
    cmdVxLog = new DoubleLogEntry(DataLogManager.getLog(), "PID/Swerve/Chassis/VxCmdMps");
    cmdVyLog = new DoubleLogEntry(DataLogManager.getLog(), "PID/Swerve/Chassis/VyCmdMps");
    cmdOmegaLog = new DoubleLogEntry(DataLogManager.getLog(), "PID/Swerve/Chassis/OmegaCmdRadPerSec");
    measVxLog = new DoubleLogEntry(DataLogManager.getLog(), "PID/Swerve/Chassis/VxMeasMps");
    measVyLog = new DoubleLogEntry(DataLogManager.getLog(), "PID/Swerve/Chassis/VyMeasMps");
    measOmegaLog = new DoubleLogEntry(DataLogManager.getLog(), "PID/Swerve/Chassis/OmegaMeasRadPerSec");

    flDriveSetpointLog = new DoubleLogEntry(DataLogManager.getLog(), "PID/Swerve/FL/DriveSetpointMps");
    flDriveMeasuredLog = new DoubleLogEntry(DataLogManager.getLog(), "PID/Swerve/FL/DriveMeasuredMps");
    flTurnSetpointLog = new DoubleLogEntry(DataLogManager.getLog(), "PID/Swerve/FL/TurnSetpointRad");
    flTurnMeasuredLog = new DoubleLogEntry(DataLogManager.getLog(), "PID/Swerve/FL/TurnMeasuredRad");

    frDriveSetpointLog = new DoubleLogEntry(DataLogManager.getLog(), "PID/Swerve/FR/DriveSetpointMps");
    frDriveMeasuredLog = new DoubleLogEntry(DataLogManager.getLog(), "PID/Swerve/FR/DriveMeasuredMps");
    frTurnSetpointLog = new DoubleLogEntry(DataLogManager.getLog(), "PID/Swerve/FR/TurnSetpointRad");
    frTurnMeasuredLog = new DoubleLogEntry(DataLogManager.getLog(), "PID/Swerve/FR/TurnMeasuredRad");

    rlDriveSetpointLog = new DoubleLogEntry(DataLogManager.getLog(), "PID/Swerve/RL/DriveSetpointMps");
    rlDriveMeasuredLog = new DoubleLogEntry(DataLogManager.getLog(), "PID/Swerve/RL/DriveMeasuredMps");
    rlTurnSetpointLog = new DoubleLogEntry(DataLogManager.getLog(), "PID/Swerve/RL/TurnSetpointRad");
    rlTurnMeasuredLog = new DoubleLogEntry(DataLogManager.getLog(), "PID/Swerve/RL/TurnMeasuredRad");

    rrDriveSetpointLog = new DoubleLogEntry(DataLogManager.getLog(), "PID/Swerve/RR/DriveSetpointMps");
    rrDriveMeasuredLog = new DoubleLogEntry(DataLogManager.getLog(), "PID/Swerve/RR/DriveMeasuredMps");
    rrTurnSetpointLog = new DoubleLogEntry(DataLogManager.getLog(), "PID/Swerve/RR/TurnSetpointRad");
    rrTurnMeasuredLog = new DoubleLogEntry(DataLogManager.getLog(), "PID/Swerve/RR/TurnMeasuredRad");
    
    
    // Configure AutoBuilder last
    AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> drive(speeds.vxMetersPerSecond,speeds.vyMetersPerSecond,speeds.omegaRadiansPerSecond,false), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(0.04, 0, 0), // Translation PID constants
                    new PIDConstants(1, 0, 0) // Rotation PID constants
            ),//-41.129
            config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    Pose2d robPose = m_driveEstimator.update(
        m_gyro.getRotation2d(),
        getModulePositions());
    
    // Do this in either robot periodic or subsystem periodic
    m_field.setRobotPose(robPose);

    SmartDashboard.putData("Field", m_field);
    SmartDashboard.putNumber("gyro Heading", getHeading()); //keeps counting past 180
    SmartDashboard.putNumber("gyro Angle", m_gyro.getRotation2d().getDegrees());
    SmartDashboard.putNumber("X Pose", getPose().getX());
    SmartDashboard.putNumber("Y Pose", getPose().getY());
    SmartDashboard.putNumber("gyro Pose Angle", getPose().getRotation().getDegrees());
    SmartDashboard.putNumber("rightFront", m_frontRight.getPosition().angle.getDegrees());
    SmartDashboard.putNumber("leftFront", m_frontLeft.getPosition().angle.getDegrees());
    SmartDashboard.putNumber("rightrear", m_rearRight.getPosition().angle.getDegrees());
    SmartDashboard.putNumber("leftrear", m_rearLeft.getPosition().angle.getDegrees());
    SmartDashboard.putNumber("Robot Gyro velocity", Math.hypot(m_gyro.getVelocityX(), m_gyro.getVelocityY()));
    SmartDashboard.putNumber("Robot Wheel velocity", Math.hypot(getRobotRelativeSpeeds().vxMetersPerSecond, getRobotRelativeSpeeds().vyMetersPerSecond));
     ChassisSpeeds measuredSpeeds = getRobotRelativeSpeeds();
    SwerveModuleState flState = m_frontLeft.getState();
    SwerveModuleState frState = m_frontRight.getState();
    SwerveModuleState rlState = m_rearLeft.getState();
    SwerveModuleState rrState = m_rearRight.getState();
    SwerveModuleState flDesired = m_frontLeft.getDesiredState();
    SwerveModuleState frDesired = m_frontRight.getDesiredState();
    SwerveModuleState rlDesired = m_rearLeft.getDesiredState();
    SwerveModuleState rrDesired = m_rearRight.getDesiredState();

    cmdVxLog.append(m_lastCmdVx);
    cmdVyLog.append(m_lastCmdVy);
    cmdOmegaLog.append(m_lastCmdOmega);
    measVxLog.append(measuredSpeeds.vxMetersPerSecond);
    measVyLog.append(measuredSpeeds.vyMetersPerSecond);
    measOmegaLog.append(measuredSpeeds.omegaRadiansPerSecond);

    flDriveSetpointLog.append(flDesired.speedMetersPerSecond);
    flDriveMeasuredLog.append(flState.speedMetersPerSecond);
    flTurnSetpointLog.append(flDesired.angle.getRadians());
    flTurnMeasuredLog.append(flState.angle.getRadians());

    frDriveSetpointLog.append(frDesired.speedMetersPerSecond);
    frDriveMeasuredLog.append(frState.speedMetersPerSecond);
    frTurnSetpointLog.append(frDesired.angle.getRadians());
    frTurnMeasuredLog.append(frState.angle.getRadians());

    rlDriveSetpointLog.append(rlDesired.speedMetersPerSecond);
    rlDriveMeasuredLog.append(rlState.speedMetersPerSecond);
    rlTurnSetpointLog.append(rlDesired.angle.getRadians());
    rlTurnMeasuredLog.append(rlState.angle.getRadians());

    rrDriveSetpointLog.append(rrDesired.speedMetersPerSecond);
    rrDriveMeasuredLog.append(rrState.speedMetersPerSecond);
    rrTurnSetpointLog.append(rrDesired.angle.getRadians());
    rrTurnMeasuredLog.append(rrState.angle.getRadians());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_driveEstimator.getEstimatedPosition();
  }

  /**
   * updates/resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_driveEstimator.resetPosition(
        m_gyro.getRotation2d(),
        getModulePositions(),
        pose);
  }

  public void resetOdometry() {
    m_driveEstimator.resetPosition(
        m_gyro.getRotation2d(),
        getModulePositions(),
        getPose());
  }
  public ChassisSpeeds getRobotRelativeSpeeds(){
    return DriveConstants.kDriveKinematics.toChassisSpeeds(
      new SwerveModuleState[] {
          m_frontLeft.getState(),
          m_frontRight.getState(),
          m_rearLeft.getState(),
          m_rearRight.getState()
      });
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.In Radians per second.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;
    m_lastCmdVx = xSpeedDelivered;
    m_lastCmdVy = ySpeedDelivered;
    m_lastCmdOmega = rotDelivered;
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                Rotation2d.fromDegrees(-m_gyro.getAngle()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    //SmartDashboard.putNumber("modstates0pre", swerveModuleStates[0].speedMetersPerSecond);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    //SmartDashboard.putNumber("modstates0post", swerveModuleStates[0].speedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
    // SmartDashboard.putNumber("m_frontLeft", m_frontLeft.getState().speedMetersPerSecond);
    // SmartDashboard.putNumber("m_frontRight", m_frontRight.getState().speedMetersPerSecond);
    // SmartDashboard.putNumber("m_rearLeft", m_rearLeft.getState().speedMetersPerSecond);
    // SmartDashboard.putNumber("m_rearRight", m_rearRight.getState().speedMetersPerSecond);
    // SmartDashboard.putNumber("m_frontLeftstate", swerveModuleStates[0].speedMetersPerSecond);
    // SmartDashboard.putNumber("m_frontRightstate", swerveModuleStates[1].speedMetersPerSecond);
    // SmartDashboard.putNumber("m_rearLeftstate", swerveModuleStates[2].speedMetersPerSecond);
    // SmartDashboard.putNumber("m_rearRightstate", swerveModuleStates[3].speedMetersPerSecond);
  }
  public void driveRobotRelative(double xSpeed, double ySpeed, double rot){
    drive(xSpeed,ySpeed,rot,false);
  }

  public Command driveCommand(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    return this.run(() -> drive(xSpeed,ySpeed,rot,true))
      .finallyDo(() -> drive(0,0,0,true));
  }

  public void turnToRotationFnc(double targetDegrees){
    double output = PIDTurnGyro.calculate(m_gyro.getRotation2d().getDegrees(), targetDegrees);
    System.out.println("output: " + output + "gyro ang: " + m_gyro.getRotation2d().getDegrees() + "target deg: " + targetDegrees);
    drive(0, 0, output, false);
  }

  public Command turnToRotation(double targetDegrees) {
    return this.run(() -> {turnToRotationFnc(targetDegrees);})
    .until(() -> PIDTurnGyro.atSetpoint())
    .finallyDo(() -> drive(0, 0, 0, false));
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  public void bumper(double speed,double rot) {
    /*
     * used to set the wheels to move slowly in a certain direction
     * AKA robot oriented drive
     * much less complicated version of the .drive method
    */
    m_frontLeft.setDesiredState(new SwerveModuleState(speed, Rotation2d.fromDegrees(rot)));
    m_frontRight.setDesiredState(new SwerveModuleState(speed, Rotation2d.fromDegrees(rot)));
    m_rearLeft.setDesiredState(new SwerveModuleState(speed, Rotation2d.fromDegrees(rot)));
    m_rearRight.setDesiredState(new SwerveModuleState(speed, Rotation2d.fromDegrees(rot)));

  }
  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void driveResetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180 <- that is incorrect it keeps counting past 180
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }
  public void addVisionMeasurement(Pose2d visionMeasurement, double timestampSeconds) {
        m_driveEstimator.addVisionMeasurement(visionMeasurement, timestampSeconds);
    }

    /** See {@link SwerveDrivePoseEstimator#addVisionMeasurement(Pose2d, double, Matrix)}. */
    public void addVisionMeasurement(
            Pose2d visionMeasurement, double timestampSeconds, Matrix<N3, N1> stdDevs) {
        m_driveEstimator.addVisionMeasurement(visionMeasurement, timestampSeconds, stdDevs);
    }
  public SwerveModulePosition[] getModulePositions() {
      return new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_rearLeft.getPosition(),
        m_rearRight.getPosition()
      };
    }
  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
      m_frontLeft.getState(),
      m_frontRight.getState(),
      m_rearLeft.getState(),
      m_rearRight.getState()  
    };
  }
  
}
