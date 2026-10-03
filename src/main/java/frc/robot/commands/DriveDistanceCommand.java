package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveDistanceCommand extends Command {
  private final DriveSubsystem m_drive;
  private final double m_xMeters;
  private final double m_yMeters;
  private final double m_targetSpeed;
  private Pose2d m_startPose;
  private double targetDistance;

  public DriveDistanceCommand(DriveSubsystem drive, double xMeters, double yMeters, double targetSpeedMetersPerSecond) {
    m_drive = drive;
    m_xMeters = xMeters;
    m_yMeters = yMeters;
    m_targetSpeed = targetSpeedMetersPerSecond;
    targetDistance = Math.hypot(xMeters, yMeters);
    addRequirements(m_drive);
  }

  @Override
  public void initialize() {
    m_startPose = m_drive.getPose();
    SmartDashboard.putNumber("DriveDistance/TargetDistance", targetDistance);
    SmartDashboard.putNumber("DriveDistance/StartX", m_startPose.getX());
    SmartDashboard.putNumber("DriveDistance/StartY", m_startPose.getY());
    SmartDashboard.putNumber("DriveDistance/RequestedX", m_xMeters);
    SmartDashboard.putNumber("DriveDistance/RequestedY", m_yMeters);
    SmartDashboard.putBoolean("DriveDistance/ReachedTarget", false);
  }

  @Override
  public void execute() {
    double maxSpeed = Constants.DriveConstants.kMaxSpeedMetersPerSecond;
    double xCommand = Math.abs(m_xMeters) > 1e-9 ? Math.copySign(m_targetSpeed / maxSpeed, m_xMeters) : 0.0;
    double yCommand = Math.abs(m_yMeters) > 1e-9 ? Math.copySign(m_targetSpeed / maxSpeed, m_yMeters) : 0.0;
    m_drive.drive(xCommand, yCommand, 0.0, false);

    double dx = m_drive.getPose().getX() - m_startPose.getX();
    double dy = m_drive.getPose().getY() - m_startPose.getY();
    double traveled = Math.hypot(dx, dy);
    SmartDashboard.putNumber("DriveDistance/ActualDistance", traveled);
    SmartDashboard.putNumber("DriveDistance/DeltaX", dx);
    SmartDashboard.putNumber("DriveDistance/DeltaY", dy);
  }

  @Override
  public boolean isFinished() {
    double dx = m_drive.getPose().getX() - m_startPose.getX();
    double dy = m_drive.getPose().getY() - m_startPose.getY();
    double traveled = Math.hypot(dx, dy);
    boolean reached = traveled >= targetDistance;
    SmartDashboard.putBoolean("DriveDistance/ReachedTarget", reached);
    return reached;
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.drive(0.0, 0.0, 0.0, false);
    SmartDashboard.putNumber("DriveDistance/FinalDistance", Math.hypot(
        m_drive.getPose().getX() - m_startPose.getX(),
        m_drive.getPose().getY() - m_startPose.getY()));
    SmartDashboard.putBoolean("DriveDistance/ReachedTarget", true);
  }
}
