package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

public class TurnToTagCommand extends Command {
    private final DriveSubsystem drive;
    private final VisionSubsystem vision;
    private final PIDController pidController;
    
    // PID constants (same as DriveSubsystem)
    private static final double kP = 0.03;
    private static final double kI = 0.00;
    private static final double kD = 0.00;
    
    public TurnToTagCommand(DriveSubsystem drive, VisionSubsystem vision) {
        this.drive = drive;
        this.vision = vision;
        this.pidController = new PIDController(kP, kI, kD);
        this.pidController.setTolerance(5.0);  // tolerance in degrees
        this.pidController.enableContinuousInput(-180, 180);  // for continuous rotation
        
        addRequirements(drive);  // Only DriveSubsystem, since this command controls driving
    }
    
    @Override
    public void initialize() {
        // Reset PID controller when command starts
        pidController.reset();
    }
    
    @Override
    public void execute() {
        double yawToTag = -vision.getYaw();
        if (yawToTag != 0) {  // Only control if we have a valid yaw
            // Error is yawToTag (we want yaw to be 0, meaning we're aligned with the tag)
            double output = pidController.calculate(0, yawToTag);
            drive.drive(0, 0, output, false);  // Robot-relative rotation only
            System.out.println("TurnToTag: yaw=" + yawToTag + " output=" + output);
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        // Stop the robot when command ends
        drive.drive(0, 0, 0, false);
    }
    
    @Override
    public boolean isFinished() {
        // Check if PID controller is at setpoint (yaw error is within tolerance)
        return pidController.atSetpoint();
    }
}
