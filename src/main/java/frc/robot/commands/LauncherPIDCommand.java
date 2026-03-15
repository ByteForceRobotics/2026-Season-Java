package frc.robot.commands;

import frc.robot.subsystems.LauncherSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Command to PID control all launcher motors to reach target RPM.
 * Controls top-left, top-right, bottom-top, and bottom-bottom motors independently.
 */
public class LauncherPIDCommand extends Command {
    private final LauncherSubsystem launcher;
    private final double targetTopRPM;
    private final double targetBottomRPM;
    private final PIDController topLeftPIDController;
    private final PIDController topRightPIDController;
    private final PIDController bottomTopPIDController;
    private final PIDController bottomBottomPIDController;
    
    // PID constants for launcher RPM control
    private static final double kP = 0.0001;
    private static final double kI = 0.00001;
    private static final double kD = 0.0001;
    private static final double kTolerance = 50.0;  // RPM tolerance
    
    /**
     * Creates a new LauncherPIDCommand to control launcher to target RPM.
     * 
     * @param launcher The LauncherSubsystem instance
     * @param targetTopRPM Target RPM for top launcher
     * @param targetBottomRPM Target RPM for bottom launcher
     */
    public LauncherPIDCommand(LauncherSubsystem launcher, double targetTopRPM, double targetBottomRPM) {
        this.launcher = launcher;
        this.targetTopRPM = targetTopRPM;
        this.targetBottomRPM = targetBottomRPM;
        
        // Create individual PID controllers for each motor
        this.topLeftPIDController = new PIDController(kP, kI, kD);
        this.topLeftPIDController.setTolerance(kTolerance);
        
        this.topRightPIDController = new PIDController(kP, kI, kD);
        this.topRightPIDController.setTolerance(kTolerance);
        
        this.bottomTopPIDController = new PIDController(kP, kI, kD);
        this.bottomTopPIDController.setTolerance(kTolerance);
        
        this.bottomBottomPIDController = new PIDController(kP, kI, kD);
        this.bottomBottomPIDController.setTolerance(kTolerance);
        
        addRequirements(launcher);  // This command requires the LauncherSubsystem
    }
    
    /**
     * Creates a new LauncherPIDCommand with same target RPM for both launchers.
     * 
     * @param launcher The LauncherSubsystem instance
     * @param targetRPM Target RPM for both launchers
     */
    public LauncherPIDCommand(LauncherSubsystem launcher, double targetRPM) {
        this(launcher, targetRPM, targetRPM);
    }
    
    @Override
    public void initialize() {
        // Reset all PID controllers when command starts
        topLeftPIDController.reset();
        topRightPIDController.reset();
        bottomTopPIDController.reset();
        bottomBottomPIDController.reset();
        System.out.println("LauncherPID: Starting control to Top=" + targetTopRPM + " RPM, Bottom=" + targetBottomRPM + " RPM");
    }
    
    @Override
    public void execute() {
        // Get current RPM values for each motor
        // Note: We need to add getter methods to LauncherSubsystem for individual motor RPMs
        // For now, we'll use the average values and control towards target
        double currentTopLeftRPM = launcher.getTopLeftRPM();
        double currentTopRightRPM = launcher.getTopRightRPM();
        double currentBottomTopRPM = launcher.getBottomTopRPM();
        double currentBottomBottomRPM = launcher.getBottomBottomRPM();
        
        // Calculate PID outputs for each motor (error is target - current)
        double topLeftOutput = topLeftPIDController.calculate(currentTopLeftRPM, targetTopRPM);
        double topRightOutput = topRightPIDController.calculate(currentTopRightRPM, targetTopRPM);
        double bottomTopOutput = bottomTopPIDController.calculate(currentBottomTopRPM, targetBottomRPM);
        double bottomBottomOutput = bottomBottomPIDController.calculate(currentBottomBottomRPM, targetBottomRPM);
        
        // Clamp outputs to [-1, 1] for motor speed
        topLeftOutput = Math.max(-1.0, Math.min(1.0, topLeftOutput));
        topRightOutput = Math.max(-1.0, Math.min(1.0, topRightOutput));
        bottomTopOutput = Math.max(-1.0, Math.min(1.0, bottomTopOutput));
        bottomBottomOutput = Math.max(-1.0, Math.min(1.0, bottomBottomOutput));
        
        // Control each motor individually
        launcher.launchTopLeftRPM(currentTopLeftRPM + topLeftOutput * 1000);
        launcher.launchTopRightRPM(currentTopRightRPM + topRightOutput * 1000);
        launcher.launchBottomTopRPM(currentBottomTopRPM + bottomTopOutput * 1000);
        launcher.launchBottomBottomRPM(currentBottomBottomRPM + bottomBottomOutput * 1000);
        
        // Debug logging
        System.out.println("LauncherPID: TopLeft=" + String.format("%.0f", currentTopLeftRPM) + 
                          " TopRight=" + String.format("%.0f", currentTopRightRPM) +
                          " BottomTop=" + String.format("%.0f", currentBottomTopRPM) + 
                          " BottomBottom=" + String.format("%.0f", currentBottomBottomRPM));
    }
    
    @Override
    public void end(boolean interrupted) {
        // Stop the launcher when command ends
        launcher.launch_stop();
        if (interrupted) {
            System.out.println("LauncherPID: Command interrupted");
        } else {
            System.out.println("LauncherPID: Command finished at Top=" + String.format("%.0f", launcher.getTopRPM()) + 
                              " RPM, Bottom=" + String.format("%.0f", launcher.getBottomRPM()) + " RPM");
        }
    }
    
    @Override
    public boolean isFinished() {
        // Command finishes when all 4 motors reach their target RPM
        return topLeftPIDController.atSetpoint() && 
               topRightPIDController.atSetpoint() && 
               bottomTopPIDController.atSetpoint() && 
               bottomBottomPIDController.atSetpoint();
    }
}
