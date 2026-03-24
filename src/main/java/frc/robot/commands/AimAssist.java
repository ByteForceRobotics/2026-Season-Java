package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;

public class AimAssist extends Command {
    private final DriveSubsystem drive;
    private final VisionSubsystem vision;
    private final PIDController pidController;
    

    //joystick values
    private final DoubleSupplier translationX;
    private final DoubleSupplier translationY;


    // PID constants (same as DriveSubsystem)
    private static final double kP = 0.03;
    private static final double kI = 0.00;
    private static final double kD = 0.00;
    
    

    public AimAssist(DriveSubsystem drive, VisionSubsystem vision, DoubleSupplier x, DoubleSupplier y) {
        this.drive = drive;
        this.vision = vision;
        this.translationX = x;
        this.translationY = y;

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
        double rotationOutput = 0;
        if (vision.hasTarget()){
            double yawToTag = -vision.getYaw();
            rotationOutput = pidController.calculate(0, yawToTag);
        }
        else{
            rotationOutput = 0;
        }
        drive.drive(translationX.getAsDouble(), translationY.getAsDouble(), rotationOutput, true);
    }
    
    @Override
    public void end(boolean interrupted) {
        // Stop the robot when command ends
        drive.drive(0, 0, 0, false);
    }
    
    @Override
    public boolean isFinished() {
        // Check if PID controller is at setpoint (yaw error is within tolerance)
        return false;
    }
}