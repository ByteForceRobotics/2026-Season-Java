package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.AgitatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LifterSubsystem;

/**
 * Command to move the intake lifter to a target position using a PID controller.
 */
public class GoToPositionLifterCommand extends Command {
    private final LifterSubsystem lifter;
    private final IntakeSubsystem intake;
    private final AgitatorSubsystem agitator;
    private final double targetPosition;
    private final PIDController pidController;
    
    // PID constants for lifter position control
    private static final double kP = IntakeConstants.kP;
    private static final double kI = IntakeConstants.kI;
    private static final double kD = IntakeConstants.kD;
    private static final double kTolerance = IntakeConstants.kTolerance;
    
    /**
     * Creates a new GoToPositionCommand for the intake lifter.
     * 
     * @param lifter The LifterSubsystem instance
     * @param intake The IntakeSubsystem instance
     * @param agitator The AgitatorSubsystem instance
     * @param targetPosition Target position in rotations (encoder counts)
     */
    public GoToPositionLifterCommand(LifterSubsystem lifter, IntakeSubsystem intake, AgitatorSubsystem agitator, double targetPosition) {
        this.lifter = lifter;
        this.intake = intake;
        this.agitator = agitator;
        this.targetPosition = targetPosition;
        this.pidController = new PIDController(kP, kI, kD);
        this.pidController.setTolerance(kTolerance);
        
        addRequirements(lifter);
    }
    
    @Override
    public void initialize() {
        if(targetPosition==0){
            intake.intake(0.2);
            agitator.agitateIntake_stop();
        }
        else{
            intake.intake(IntakeConstants.kIntakeDefaultSpeed);
            agitator.agitateIntake();
        }
        // Reset PID controller when command starts
        pidController.reset();
        //System.out.println("GoToPosition: Moving lifter to position " + targetPosition);
    }
    
    @Override
    public void execute() {
        double newP = SmartDashboard.getNumber("Lifter/PID/kP", kP);
        double newI = SmartDashboard.getNumber("Lifter/PID/kI", kI);
        double newD = SmartDashboard.getNumber("Lifter/PID/kD", kD);
        double newTolerance = SmartDashboard.getNumber("Launcher/kTolerance", kTolerance);
        //System.out.println("GoToPosition: Updated PID constants from SmartDashboard - P: " + newP + " I: " + newI + " D: " + newD + " Tolerance: " + newTolerance);
        this.updatePIDConstants(newP, newI, newD, newTolerance);

        double currentPosition = lifter.getLifterPosition();
        
        // Calculate PID output (error is target - current)
        double output = pidController.calculate(currentPosition, targetPosition);
        
        // Clamp output to [-1, 1] for motor speed
        output = Math.max(-1.0, Math.min(1.0, output));
        
        lifter.lift(output);
        
        
        // Debug logging
        //System.out.println("GoToPosition: current=" + currentPosition + " target=" + targetPosition + " output=" + output);
    }
    
    @Override
    public void end(boolean interrupted) {
        // Stop the lifter when command ends
        lifter.lift_stop();
        if (interrupted) {
            System.out.println("GoToPosition: Command interrupted at position " + lifter.getLifterPosition());
        } else {
            System.out.println("GoToPosition: Command finished at position " + lifter.getLifterPosition());
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
