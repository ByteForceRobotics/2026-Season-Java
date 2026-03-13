package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Command to move the intake lifter to a target position using a PID controller.
 */
public class GoToPositionLifterCommand extends Command {
    private final IntakeSubsystem intake;
    private final double targetPosition;
    private final PIDController pidController;
    
    // PID constants for lifter position control
    private static final double kP = 0.5;   // Proportional gain
    private static final double kI = 0.0;   // Integral gain
    private static final double kD = 0.1;   // Derivative gain
    private static final double kTolerance = 0.05;  // Position tolerance in rotations
    
    /**
     * Creates a new GoToPositionCommand for the intake lifter.
     * 
     * @param intake The IntakeSubsystem instance
     * @param targetPosition Target position in rotations (encoder counts)
     */
    public GoToPositionLifterCommand(IntakeSubsystem intake, double targetPosition) {
        this.intake = intake;
        this.targetPosition = targetPosition;
        this.pidController = new PIDController(kP, kI, kD);
        this.pidController.setTolerance(kTolerance);
        
        addRequirements(intake);  // This command requires the IntakeSubsystem
    }
    
    @Override
    public void initialize() {
        // Reset PID controller when command starts
        pidController.reset();
        System.out.println("GoToPosition: Moving lifter to position " + targetPosition);
    }
    
    @Override
    public void execute() {
        double currentPosition = intake.getLifterPosition();
        
        // Calculate PID output (error is target - current)
        double output = pidController.calculate(currentPosition, targetPosition);
        
        // Clamp output to [-1, 1] for motor speed
        output = Math.max(-1.0, Math.min(1.0, output));
        
        intake.lift(output);
        
        // Debug logging
        System.out.println("GoToPosition: current=" + currentPosition + " target=" + targetPosition + " output=" + output);
    }
    
    @Override
    public void end(boolean interrupted) {
        // Stop the lifter when command ends
        intake.lift_stop();
        if (interrupted) {
            System.out.println("GoToPosition: Command interrupted at position " + intake.getLifterPosition());
        } else {
            System.out.println("GoToPosition: Command finished at position " + intake.getLifterPosition());
        }
    }
    
    @Override
    public boolean isFinished() {
        // Command finishes when PID controller reaches setpoint
        return pidController.atSetpoint();
    }
}
