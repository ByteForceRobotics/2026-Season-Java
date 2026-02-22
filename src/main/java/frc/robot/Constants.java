// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
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
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 3;// Earlier max was 4.8
    public static final double kMaxAngularSpeed = 1.2 * Math.PI; // radians per second

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
    public static final int kClimbCanId = 9;
    public static final int kClimbFollowerCanId = 10;//might not need
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
    public static final int kIntakeCurrentLimit = 30;//set this
    public static final int kIntakeLifterCurrentLimit = 40;//set this
    public static final IdleMode kIntakeIdleMode = IdleMode.kCoast;
    public static final IdleMode kLifterIdleMode = IdleMode.kBrake;
    public static final double kLifterMaxHeight = 4; //set this
    public static final double kIntakeDefaultSpeed = 0.6;
    public static final double kLiftDefaultSpeed = 0.15;
  }
  
  public static final class LauncherConstants {
    public static final int kLauncher1CanId = 13;//top
    public static final int kLauncher2CanId = 14;//bottom
    public static final int kLauncher1CurrentLimit = 30;//set this
    public static final int kLauncher2CurrentLimit = 30;//set this
    public static final IdleMode kLauncherIdleMode = IdleMode.kBrake;
    public static final double kLauncherDefaultSpeed = 0.6;
  }
  
  public static final class AgitatorConstants{
    public static final int kAgitatorCanId = 15;
    public static final int kAgitatorCurrentLimit = 30;//set this
    public static final IdleMode kAgitatorIdleMode = IdleMode.kCoast;
    public static final double kAgitatorDefaultSpeed = 1;//set this
  }

  public static final class CameraConstants {
        
    public static final String kCameraName = "FrontCam";
        // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
        public static final Transform3d kRobotToCam =
                new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));

        // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout kTagLayout =
                AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

        // The standard deviations of our vision estimated poses, which affect correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
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
