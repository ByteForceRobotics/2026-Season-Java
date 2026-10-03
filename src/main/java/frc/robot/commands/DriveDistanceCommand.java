package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class DriveDistanceCommand extends Command {
  private final DriveSubsystem m_drive;
  private final double m_xMeters;
  private final double m_yMeters;
  private final double m_targetSpeed;
  private Pose2d m_startPose;

  public DriveDistanceCommand(DriveSubsystem drive, double xMeters, double yMeters, double targetSpeed) {
    m_drive = drive;
    m_xMeters = xMeters;
    m_yMeters = yMeters;
    m_targetSpeed = targetSpeed;
    addRequirements(m_drive);
  }

  @Override
  public void initialize() {
    m_startPose = m_drive.getPose();
  }

  @Override
  public void execute() {
    double xCommand = Math.abs(m_xMeters) > 1e-9 ? Math.copySign(m_targetSpeed, m_xMeters) : 0.0;
    double yCommand = Math.abs(m_yMeters) > 1e-9 ? Math.copySign(m_targetSpeed, m_yMeters) : 0.0;
    m_drive.drive(xCommand, yCommand, 0.0, false);
  }

  @Override
  public boolean isFinished() {
    double dx = Math.abs(m_drive.getPose().getX() - m_startPose.getX());
    double dy = Math.abs(m_drive.getPose().getY() - m_startPose.getY());

    if (Math.abs(m_xMeters) > 1e-9) {
      return dx >= Math.abs(m_xMeters);
    }
    if (Math.abs(m_yMeters) > 1e-9) {
      return dy >= Math.abs(m_yMeters);
    }
    return true;
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.drive(0.0, 0.0, 0.0, false);
  }
}
