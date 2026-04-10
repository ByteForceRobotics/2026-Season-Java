// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    
    public static final boolean kSysID_characterization_enable = false; // keep false unless characterizing PID etc
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 3;// Earlier max was 4.8
    public static final double kMaxAngularSpeed = 1.2* Math.PI; // radians per second

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(23.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(23.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 2;
    public static final int kRearLeftDrivingCanId = 4;
    public static final int kFrontRightDrivingCanId = 6;
    public static final int kRearRightDrivingCanId = 8;

    public static final int kFrontLeftTurningCanId = 1;
    public static final int kRearLeftTurningCanId = 3;
    public static final int kFrontRightTurningCanId = 5;
    public static final int kRearRightTurningCanId = 7;


    public static final boolean kGyroReversed = false;
  }

  public static final class ClimbConstants {
    public static final int kClimbCanId = -1;
    public static final int kClimbFollowerCanId = -1;//might not need
    public static final int kClimbCurrentLimit = 30;
    public static final IdleMode kClimbIdleMode = IdleMode.kBrake;
    public static final double kP = 1;
    public static final double kI = 1;
    public static final double kD = 1;
    public static final double kClimbLowerLimit = 35.0;//need to figure this out
    public static final double kMaxRetract = 35.0;//this one too
    public static final double kMaxExtend = 0;//this one as well
    public static final double kClimbSpeed = 0.5;// figure this out
    public static final double kLevelOneTime = 1.0;//seconds, figure this one out

  }

  public static final class IntakeConstants {
    public static final int kIntakeLifterCanId = 11;
    public static final int kIntakeCanId = 12;
    public static final int kIntakeCurrentLimit = 40;//set this
    public static final int kIntakeLifterCurrentLimit = 40;//set this
    public static final IdleMode kIntakeIdleMode = IdleMode.kCoast;
    public static final IdleMode kLifterIdleMode = IdleMode.kCoast;
    public static final double kLifterMaxLower = 0.33;
    public static final double kLifterMaxLift = 0; //initialize absolute encoder when lifted to zero
    public static final double kIntakeDefaultSpeed = 0.60;
    public static final double kLiftDefaultSpeed = 0.3;
    public static final double kP = 1;   // Proportional gain
    public static final double kI = 0.01;   // Integral gain  
    public static final double kD = 0.1;   // Derivative gain
    public static final double kTolerance = 0.001; 
  }
  
  public static final class LauncherConstants {
    public static final boolean kManualControl = true;//true uses smartdashboard values, false uses interpolation/defualt values
    public static final int kLauncherTopLeftCanId = 13;
    public static final int kLauncherTopRightCanId = 14;
    public static final int kLauncherBottomTopCanId = 15;//bottom
    public static final int kLauncherBottomBottomCanId = 16;//bottom
    public static final int kLauncher1CurrentLimit = 80;
    public static final int kLauncher2CurrentLimit = 80;
    public static final IdleMode kLauncherTopIdleMode = IdleMode.kCoast;
    public static final IdleMode kLauncherBottomIdleMode = IdleMode.kBrake;
    public static final double kLauncherSpeed = 0.65;
    public static final double kTopP = 0.0006;
    public static final double kTopI = 0.0;
    public static final double kTopD = 0.03;
    public static final double kTopTolerance = 50;  // RPM tolerance
    public static final double kBottomTopP = 0.0002;//test these
    public static final double kBottomTopI =  0.0000001;
    public static final double kBottomTopD = 0;
    public static final double kBottomBottomP = 0.0002;//test these
    public static final double kBottomBottomI = 0.0000001;
    public static final double kBottomBottomD = 0.0000;//,maybe figure out a small value
    public static final double kBottomTolerance = 0;  // RPM tolerance
    public static final double kLauncherDefaultBottomRPM = 800;
    public static final double kLauncherDefaultTopRPM = 3800;
    public static final double kBottomLauncherDelay = 0.8;//max distance we can shoot from, in meters
    //interpolation is credited to team 2059 hitchhikers
    public static final InterpolatingTreeMap<Double, ShooterParams> SHOOTER_MAP = new InterpolatingTreeMap<>(
		  InverseInterpolator.forDouble(),

		  // Value interpolator: blends RPMs and flight times based on distance ratio, t
		  (start, end, t) -> new ShooterParams(
			MathUtil.interpolate(start.rpm(), end.rpm(), t),
			MathUtil.interpolate(start.timeOfFlight(), end.timeOfFlight(), t)
		  )
		);
    static {
			//DISTANCE FROM CENTER OF SHOOTER TO CENTER OF HUB, IN METERS
      //distance meters, rpm, time of flight seconds
      //time of flight is only needed for shooting while moving
      SHOOTER_MAP.put(1.89, new ShooterParams(3130, 0.68));
      SHOOTER_MAP.put(2.28, new ShooterParams(3240, 0.68));
			 SHOOTER_MAP.put(3.37, new ShooterParams(3550, 0.68));
			 SHOOTER_MAP.put(4.0, new ShooterParams(3600, 0.8));
       SHOOTER_MAP.put(4.8, new ShooterParams(3780, 0.8));
			// SHOOTER_MAP.put(3.5, new ShooterParams(2950, 0.9));
			// SHOOTER_MAP.put(4.0, new ShooterParams(3150, 1.0));
			// SHOOTER_MAP.put(4.865, new ShooterParams(3550, 1.3));
			// SHOOTER_MAP.put(5.269, new ShooterParams(3800, 1.34));
		}



    public record ShooterParams(double rpm, double timeOfFlight) {}
  }
  
  public static final class AgitatorConstants{
    public static final int kAgitatorMainCanId = 17;
    public static final int kAgitatorIntakeCanId = 18;
    public static final int kAgitatorCurrentLimit = 40;//set this
    public static final IdleMode kAgitatorIdleMode = IdleMode.kBrake;
    public static final double kAgitatorDefaultSpeed = 0.1;
  }

  public static final class CameraConstants {
    public static final double kFrontCamHeight = 0.508;//set this
    public static final double kRearCamHeight = 0;//set this
    public static final double kFrontXOffset = 0;
    public static final double kFrontYOffset = 0;
    public static final double kFrontRotation = 0;
    public static final double kRearXOffset = 0;
    public static final double kRearYOffset = 0;
    public static final double kRearRotation = 0;

  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 13;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kClimbControllerPort = 1;
    public static final double kDriveDeadband = 0.07;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }
}
