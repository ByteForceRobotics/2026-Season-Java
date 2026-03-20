package frc.robot.commands;

import frc.robot.Constants.LauncherConstants;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Command to PID control all launcher motors to reach target RPM.
 * Controls top-left, top-right, bottom-top, and bottom-bottom motors independently.
 */
public class LauncherPIDCommand extends Command {
    private final LauncherSubsystem launcher;
    private final VisionSubsystem vision;
    private double targetTopRPM;
    private double targetBottomRPM;
    private final PIDController topLeftPIDController;
    private final PIDController topRightPIDController;
    private final PIDController bottomTopPIDController;
    private final PIDController bottomBottomPIDController;
    
    // PID constants for launcher RPM control - separate for top and bottom
    private static final double kPTop = LauncherConstants.kTopP;
    private static final double kITop = LauncherConstants.kTopI;
    private static final double kDTop = LauncherConstants.kTopD;
    private static final double kToleranceTop = LauncherConstants.kTopTolerance;
    
    private static final double kPBottom = LauncherConstants.kBottomP;
    private static final double kIBottom = LauncherConstants.kBottomI;
    private static final double kDBottom = LauncherConstants.kBottomD;
    private static final double kToleranceBottom = LauncherConstants.kBottomTolerance;
    
    /**
     * Creates a new LauncherPIDCommand to control launcher to target RPM.
     * 
     * @param launcher The LauncherSubsystem instance
     * @param targetTopRPM Target RPM for top launcher
     * @param targetBottomRPM Target RPM for bottom launcher
     */
    public LauncherPIDCommand(LauncherSubsystem launcher, VisionSubsystem vision, double targetTopRPM, double targetBottomRPM) {
        this.launcher = launcher;
        this.vision = vision;
        this.targetTopRPM = getTargetTopRPM(targetTopRPM);
        this.targetBottomRPM = getTargetBottomRPM(targetBottomRPM);
        
        // Create individual PID controllers for each motor
        // Top launcher motors use top PID constants
        this.topLeftPIDController = new PIDController(kPTop, kITop, kDTop);
        this.topLeftPIDController.setTolerance(kToleranceTop);
        
        this.topRightPIDController = new PIDController(kPTop, kITop, kDTop);
        this.topRightPIDController.setTolerance(kToleranceTop);
        
        // Bottom launcher motors use bottom PID constants
        this.bottomTopPIDController = new PIDController(kPBottom, kIBottom, kDBottom);
        this.bottomTopPIDController.setTolerance(kToleranceBottom);
        
        this.bottomBottomPIDController = new PIDController(kPBottom, kIBottom, kDBottom);
        this.bottomBottomPIDController.setTolerance(kToleranceBottom);
        
        
        addRequirements(launcher);  // This command requires the LauncherSubsystem
    }
    
    /**
     * Creates a new LauncherPIDCommand with same target RPM for both launchers.
     * 
     * @param launcher The LauncherSubsystem instance
     * @param targetRPM Target RPM for both launchers
     */
    public LauncherPIDCommand(LauncherSubsystem launcher,VisionSubsystem vision, double targetRPM) {
        this(launcher, vision, targetRPM, targetRPM);
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
        // Read separate PID constants from SmartDashboard for top and bottom
        double newPTop = SmartDashboard.getNumber("Launcher/PID/Top/kP", kPTop);
        double newITop = SmartDashboard.getNumber("Launcher/PID/Top/kI", kITop);
        double newDTop = SmartDashboard.getNumber("Launcher/PID/Top/kD", kDTop);
        double newToleranceTop = SmartDashboard.getNumber("Launcher/PID/Top/Tolerance", kToleranceTop);
        
        double newPBottom = SmartDashboard.getNumber("Launcher/PID/Bottom/kP", kPBottom);
        double newIBottom = SmartDashboard.getNumber("Launcher/PID/Bottom/kI", kIBottom);
        double newDBottom = SmartDashboard.getNumber("Launcher/PID/Bottom/kD", kDBottom);
        double newToleranceBottom = SmartDashboard.getNumber("Launcher/PID/Bottom/Tolerance", kToleranceBottom);
        
        // Update PID constants for top and bottom launchers separately
        this.updateTopPIDConstants(newPTop, newITop, newDTop, newToleranceTop);
        this.updateBottomPIDConstants(newPBottom, newIBottom, newDBottom, newToleranceBottom);
        
        this.targetTopRPM = SmartDashboard.getNumber("Launcher/GoalTopRPM",-1);
        this.targetBottomRPM = SmartDashboard.getNumber("Launcher/GoalBottomRPM",-1);
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
        
        System.out.println("LauncherPID: TopLeft=" + String.format("%.2f", currentTopLeftRPM) + 
                          " TopRight=" + String.format("%.2f", currentTopRightRPM) +
                          " BottomTop=" + String.format("%.2f", currentBottomTopRPM) + 
                          " BottomBottom=" + String.format("%.2f", currentBottomBottomRPM));
        System.out.println("LauncherPID:TargetRPM Top=" + String.format("%.2f", targetTopRPM) + 
                          " Bottom=" + String.format("%.2f", targetBottomRPM));
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
    
    /**
     * Update PID constants for top launcher motors independently.
     */
    public void updateTopPIDConstants(double kP, double kI, double kD, double kTolerance) {
        topLeftPIDController.setPID(kP, kI, kD);
        topRightPIDController.setPID(kP, kI, kD);
        topLeftPIDController.setTolerance(kTolerance);
        topRightPIDController.setTolerance(kTolerance);
    }
    
    /**
     * Update PID constants for bottom launcher motors independently.
     */
    public void updateBottomPIDConstants(double kP, double kI, double kD, double kTolerance) {
        bottomTopPIDController.setPID(kP, kI, kD);
        bottomBottomPIDController.setPID(kP, kI, kD);
        bottomTopPIDController.setTolerance(kTolerance);
        bottomBottomPIDController.setTolerance(kTolerance);
    }
    
    /**
     * Update PID constants for all launcher motors.
     */
    public void updatePIDConstants(double kP, double kI, double kD, double kTolerance) {
        topLeftPIDController.setPID(kP, kI, kD);
        topRightPIDController.setPID(kP, kI, kD);
        bottomTopPIDController.setPID(kP, kI, kD);
        bottomBottomPIDController.setPID(kP, kI, kD);
        topLeftPIDController.setTolerance(kTolerance);
        topRightPIDController.setTolerance(kTolerance);
        bottomTopPIDController.setTolerance(kTolerance);
        bottomBottomPIDController.setTolerance(kTolerance);
    }
    
    @Override
    public boolean isFinished() {
        // Command finishes when all 4 motors reach their target RPM
        return false;
    }
    public double getTargetTopRPM(double targetTopRPM){
        double targetRPM = SmartDashboard.getNumber("GoalTopRPM", -1);
        return targetRPM;
        // if(vision.getHorizDistance()<0){
        //     return targetTopRPM;
        // }
        // else{
        //     double horizDistance = vision.getHorizDistance();
        //     double targetRPM = LauncherConstants.SHOOTER_MAP.get(horizDistance).rpm();
        //     return targetRPM;
        // }
    }
    public double getTargetBottomRPM(double targetBottomRPM){
        double targetRPM = SmartDashboard.getNumber("GoalBottomRPM", -1);
        return targetRPM;
    }
}
