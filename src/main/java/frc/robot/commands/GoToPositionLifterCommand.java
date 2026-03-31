package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LauncherConstants;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Command to move the intake lifter to a target position using a PID controller.
 */
public class GoToPositionLifterCommand extends Command {
    private final IntakeSubsystem intake;
    private final double targetPosition;
    private final PIDController pidController;
    
    // PID constants for lifter position control
    private static final double kP = IntakeConstants.kP;   // Proportional gain
    private static final double kI = IntakeConstants.kI;   // Integral gain
    private static final double kD = IntakeConstants.kD;   // Derivative gain
    private static final double kTolerance = IntakeConstants.kTolerance;  // Position tolerance in rotations
    
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
        
    }
    
    @Override
    public void initialize() {
        // Reset PID controller when command starts
        pidController.reset();
        System.out.println("GoToPosition: Moving lifter to position " + targetPosition);
    }
    
    @Override
    public void execute() {
        double newP = SmartDashboard.getNumber("Lifter/PID/kP", kP);
        double newI = SmartDashboard.getNumber("Lifter/PID/kI", kI);
        double newD = SmartDashboard.getNumber("Lifter/PID/kD", kD);
        double newTolerance = SmartDashboard.getNumber("Launcher/kTolerance", kTolerance);
        System.out.println("GoToPosition: Updated PID constants from SmartDashboard - P: " + newP + " I: " + newI + " D: " + newD + " Tolerance: " + newTolerance);
        this.updatePIDConstants(newP, newI, newD, newTolerance);

        double currentPosition = intake.getLifterPosition();
        
        // Calculate PID output (error is target - current)
        double output = pidController.calculate(currentPosition, targetPosition);
        
        // Clamp output to [-1, 1] for motor speed
        output = Math.max(-1.0, Math.min(1.0, output));
        
        intake.lift(output);
        if(targetPosition==0){
            intake.intake_stop();
        }
        else{
            intake.intake(IntakeConstants.kIntakeDefaultSpeed);
        }
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
        return false;
    }
    public void updatePIDConstants(double kP, double kI, double kD, double tolerance) {
        pidController.setPID(kP, kI, kD);
        pidController.setTolerance(tolerance);
    }
}
