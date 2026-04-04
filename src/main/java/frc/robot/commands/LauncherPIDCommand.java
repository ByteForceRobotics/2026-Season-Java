package frc.robot.commands;

import frc.robot.Constants.LauncherConstants;
import frc.robot.subsystems.AgitatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import static frc.robot.Constants.LauncherConstants.kTopTolerance;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Command to PID control all launcher motors to reach target RPM.
 * Controls top-left, top-right, bottom-top, and bottom-bottom motors independently.
 */
public class LauncherPIDCommand extends Command {
    private final LauncherSubsystem launcher;
    private final VisionSubsystem vision;
    private final IntakeSubsystem intake;
    private final AgitatorSubsystem agitator;
    private double targetTopRPM;
    private double targetBottomRPM;

    // private final PIDController topLeftPIDController;
    // private final PIDController topRightPIDController;
    // private final PIDController bottomTopPIDController;
    // private final PIDController bottomBottomPIDController;
    private final DoubleLogEntry topLeftSetpointLog;
    private final DoubleLogEntry topLeftMeasuredLog;
    private final DoubleLogEntry topLeftErrorLog;
    private final DoubleLogEntry topLeftOutputLog;

    private final DoubleLogEntry topRightSetpointLog;
    private final DoubleLogEntry topRightMeasuredLog;
    private final DoubleLogEntry topRightErrorLog;
    private final DoubleLogEntry topRightOutputLog;

    private final DoubleLogEntry bottomTopSetpointLog;
    private final DoubleLogEntry bottomTopMeasuredLog;
    private final DoubleLogEntry bottomTopErrorLog;
    private final DoubleLogEntry bottomTopOutputLog;

    private final DoubleLogEntry bottomBottomSetpointLog;
    private final DoubleLogEntry bottomBottomMeasuredLog;
    private final DoubleLogEntry bottomBottomErrorLog;
    private final DoubleLogEntry bottomBottomOutputLog;

    private final BooleanLogEntry bottomActiveLog;
    
    // Timer for delaying bottom launcher spinup to avoid stutter
    private double commandStartTime = 0;
    private double bottomLauncherDelay = LauncherConstants.kBottomLauncherDelay; // Default 300ms delay (adjustable via SmartDashboard)
    
    // PID constants for launcher RPM control - separate for all motors
    // private static final double kPTop = LauncherConstants.kTopP;
    // private static final double kITop = LauncherConstants.kTopI;
    // private static final double kDTop = LauncherConstants.kTopD;
    // private static final double kToleranceTop = LauncherConstants.kTopTolerance;
    
    // private static final double kPBottomTop = LauncherConstants.kBottomTopP;
    // private static final double kIBottomTop = LauncherConstants.kBottomTopI;
    // private static final double kDBottomTop = LauncherConstants.kBottomTopD;
    // private static final double kToleranceBottomTop = LauncherConstants.kBottomTolerance;
    
    // private static final double kPBottomBottom = LauncherConstants.kBottomBottomP;
    // private static final double kIBottomBottom = LauncherConstants.kBottomBottomI;
    // private static final double kDBottomBottom = LauncherConstants.kBottomBottomD;
    // private static final double kToleranceBottomBottom = LauncherConstants.kBottomTolerance;
    
    /**
     * Creates a new LauncherPIDCommand to control launcher to target RPM.
     * 
     * @param launcher The LauncherSubsystem instance
     * @param targetTopRPM Target RPM for top launcher
     * @param targetBottomRPM Target RPM for bottom launcher
     */
    public LauncherPIDCommand(LauncherSubsystem launcher, VisionSubsystem vision,IntakeSubsystem intake,AgitatorSubsystem agitator, double targetTopRPM, double targetBottomRPM) {
        this.launcher = launcher;
        this.vision = vision;
        this.intake = intake;
        this.agitator = agitator;
        this.targetTopRPM = getTargetTopRPM(targetTopRPM);
        this.targetBottomRPM = getTargetBottomRPM(targetBottomRPM);
        
        // Create individual PID controllers for each motor
        // // Top launcher motors use top PID constants
        // this.topLeftPIDController = new PIDController(kPTop, kITop, kDTop);
        // this.topLeftPIDController.setTolerance(kToleranceTop);
        
        // this.topRightPIDController = new PIDController(kPTop, kITop, kDTop);
        // this.topRightPIDController.setTolerance(kToleranceTop);
        
        // // Bottom launcher motors use separate PID constants
        // this.bottomTopPIDController = new PIDController(kPBottomTop, kIBottomTop, kDBottomTop);
        // this.bottomTopPIDController.setTolerance(kToleranceBottomTop);
        
        // this.bottomBottomPIDController = new PIDController(kPBottomBottom, kIBottomBottom, kDBottomBottom);
        // this.bottomBottomPIDController.setTolerance(kToleranceBottomBottom);
         topLeftSetpointLog = new DoubleLogEntry(DataLogManager.getLog(), "PID/Launcher/TopLeft/SetpointRPM");
        topLeftMeasuredLog = new DoubleLogEntry(DataLogManager.getLog(), "PID/Launcher/TopLeft/MeasuredRPM");
        topLeftErrorLog = new DoubleLogEntry(DataLogManager.getLog(), "PID/Launcher/TopLeft/ErrorRPM");
        topLeftOutputLog = new DoubleLogEntry(DataLogManager.getLog(), "PID/Launcher/TopLeft/Output");
        

        topRightSetpointLog = new DoubleLogEntry(DataLogManager.getLog(), "PID/Launcher/TopRight/SetpointRPM");
        topRightMeasuredLog = new DoubleLogEntry(DataLogManager.getLog(), "PID/Launcher/TopRight/MeasuredRPM");
        topRightErrorLog = new DoubleLogEntry(DataLogManager.getLog(), "PID/Launcher/TopRight/ErrorRPM");
        topRightOutputLog = new DoubleLogEntry(DataLogManager.getLog(), "PID/Launcher/TopRight/Output");

        bottomTopSetpointLog = new DoubleLogEntry(DataLogManager.getLog(), "PID/Launcher/BottomTop/SetpointRPM");
        bottomTopMeasuredLog = new DoubleLogEntry(DataLogManager.getLog(), "PID/Launcher/BottomTop/MeasuredRPM");
        bottomTopErrorLog = new DoubleLogEntry(DataLogManager.getLog(), "PID/Launcher/BottomTop/ErrorRPM");
        bottomTopOutputLog = new DoubleLogEntry(DataLogManager.getLog(), "PID/Launcher/BottomTop/Output");

        bottomBottomSetpointLog = new DoubleLogEntry(DataLogManager.getLog(), "PID/Launcher/BottomBottom/SetpointRPM");
        bottomBottomMeasuredLog = new DoubleLogEntry(DataLogManager.getLog(), "PID/Launcher/BottomBottom/MeasuredRPM");
        bottomBottomErrorLog = new DoubleLogEntry(DataLogManager.getLog(), "PID/Launcher/BottomBottom/ErrorRPM");
        bottomBottomOutputLog = new DoubleLogEntry(DataLogManager.getLog(), "PID/Launcher/BottomBottom/Output");
        bottomActiveLog = new BooleanLogEntry(DataLogManager.getLog(), "PID/Launcher/BottomBottom/Active");
     
        
    }
    
    /**
     * Creates a new LauncherPIDCommand with same target RPM for both launchers.
     * 
     * @param launcher The LauncherSubsystem instance
     * @param targetRPM Target RPM for both launchers
     */
    
    @Override
    public void initialize() {
        // // Reset all PID controllers when command starts
        // topLeftPIDController.reset();
        // topRightPIDController.reset();
        // bottomTopPIDController.reset();
        // bottomBottomPIDController.reset();
        
        // Start timer for bottom launcher delay
        this.commandStartTime = System.currentTimeMillis() / 1000.0;
        
        // Read delay from SmartDashboard (default 0.3 seconds)
        if(LauncherConstants.kManualControl){
            this.bottomLauncherDelay = SmartDashboard.getNumber("Launcher/BottomDelay", 0.3);
        }
        else{
            this.bottomLauncherDelay = LauncherConstants.kBottomLauncherDelay;
        }
        
        
        //System.out.println("LauncherPID: Starting control to Top=" + targetTopRPM + " RPM, Bottom=" + targetBottomRPM + " RPM (bottom delay=" + bottomLauncherDelay + "s)");
    }
    
    @Override
    public void execute() {
        intake.intake_stop();
        this.targetTopRPM = getTargetTopRPM(targetTopRPM);
        this.targetBottomRPM = getTargetBottomRPM(targetBottomRPM);
        // Get current RPM values for each motor
        // Read separate PID constants from SmartDashboard for top, bottom-top, and bottom-bottom
        // double newPTop = SmartDashboard.getNumber("Launcher/PID/Top/kP", kPTop);
        // double newITop = SmartDashboard.getNumber("Launcher/PID/Top/kI", kITop);
        // double newDTop = SmartDashboard.getNumber("Launcher/PID/Top/kD", kDTop);
        // double newToleranceTop = SmartDashboard.getNumber("Launcher/PID/Top/Tolerance", kToleranceTop);
        
        // double newPBottomTop = SmartDashboard.getNumber("Launcher/PID/BottomTop/kP", kPBottomTop);
        // double newIBottomTop = SmartDashboard.getNumber("Launcher/PID/BottomTop/kI", kIBottomTop);
        // double newDBottomTop = SmartDashboard.getNumber("Launcher/PID/BottomTop/kD", kDBottomTop);
        // double newToleranceBottomTop = SmartDashboard.getNumber("Launcher/PID/BottomTop/Tolerance", kToleranceBottomTop);
        
        // double newPBottomBottom = SmartDashboard.getNumber("Launcher/PID/BottomBottom/kP", kPBottomBottom);
        // double newIBottomBottom = SmartDashboard.getNumber("Launcher/PID/BottomBottom/kI", kIBottomBottom);
        // double newDBottomBottom = SmartDashboard.getNumber("Launcher/PID/BottomBottom/kD", kDBottomBottom);
        // double newToleranceBottomBottom = SmartDashboard.getNumber("Launcher/PID/BottomBottom/Tolerance", kToleranceBottomBottom);
        
        // // Update PID constants for all motors independently
        // if(LauncherConstants.kManualControl){
        //     this.updateTopPIDConstants(newPTop, newITop, newDTop, newToleranceTop);
        //     this.updateBottomTopPIDConstants(newPBottomTop, newIBottomTop, newDBottomTop, newToleranceBottomTop);
        //     this.updateBottomBottomPIDConstants(newPBottomBottom, newIBottomBottom, newDBottomBottom, newToleranceBottomBottom);
        // }
        if(LauncherConstants.kManualControl){
            this.targetTopRPM = SmartDashboard.getNumber("Launcher/GoalTopRPM",-1);
            this.targetBottomRPM = SmartDashboard.getNumber("Launcher/GoalBottomRPM",-1);
        }
        
        double currentTopLeftRPM = launcher.getTopLeftRPM();
        double currentTopRightRPM = launcher.getTopRightRPM();
        double currentBottomTopRPM = launcher.getBottomTopRPM();
        double currentBottomBottomRPM = launcher.getBottomBottomRPM();
        
        // // Calculate PID outputs for each motor (error is target - current)
        // double topLeftOutput = topLeftPIDController.calculate(currentTopLeftRPM, targetTopRPM);
        // double topRightOutput = topRightPIDController.calculate(currentTopRightRPM, targetTopRPM);
        // double bottomTopOutput = bottomTopPIDController.calculate(currentBottomTopRPM, targetBottomRPM);
        // double bottomBottomOutput = bottomBottomPIDController.calculate(currentBottomBottomRPM, targetBottomRPM/2);
        
        // // Clamp outputs to [-1, 1] for motor speed
        // topLeftOutput = Math.max(-1.0, Math.min(1.0, topLeftOutput));
        // topRightOutput = Math.max(-1.0, Math.min(1.0, topRightOutput));
        // bottomTopOutput = Math.max(-1.0, Math.min(1.0, bottomTopOutput));
        // bottomBottomOutput = Math.max(-1.0, Math.min(1.0, bottomBottomOutput));
        
        // Check if we should spin up bottom launchers (delay to avoid stutter)
        
        // Control each motor individually
        // Top motors always spin (no delay)
        launcher.launchTopLeftRPM(targetTopRPM);
        launcher.launchTopRightRPM(targetTopRPM);
        launcher.launchBottomTopRPM(targetBottomRPM);
        //credit to hitchikers foor smart way of launching down below
        boolean readyToLaunch = ((targetTopRPM-currentTopRightRPM)<kTopTolerance)&&((targetTopRPM-currentTopLeftRPM)<kTopTolerance);
        // Bottom motors only spin after delay
        if (readyToLaunch) {
            launcher.launchBottomBottomRPM(targetBottomRPM);
            agitator.agitateMain();
            agitator.agitateIntake();
        } else {
            // Keep bottom motors at 0 RPM until delay passes
            launcher.launchBottomBottomRPM(0);
            agitator.agitateMain_stop();
            agitator.agitateIntake_stop();
        }

        double topLeftOutput = launcher.getTopLeftOutput();
        topLeftSetpointLog.append(targetTopRPM);
        topLeftMeasuredLog.append(currentTopLeftRPM);
        topLeftErrorLog.append(targetTopRPM - currentTopLeftRPM);
        topLeftOutputLog.append(topLeftOutput);
        
        // SmartDashboard.putNumber("Launcher/TopLeft/SetpointRPM", targetTopRPM);
        // SmartDashboard.putNumber("Launcher/TopLeft/MeasuredRPM", currentTopLeftRPM);
        // SmartDashboard.putNumber("Launcher/TopLeft/ErrorRPM", targetTopRPM - currentTopLeftRPM);
        //SmartDashboard.putNumber("Launcher/TopLeft/Output", topLeftOutput);

        double topRightOutput = launcher.getTopRightOutput();
        topRightSetpointLog.append(targetTopRPM);
        topRightMeasuredLog.append(currentTopRightRPM);
        topRightErrorLog.append(targetTopRPM - currentTopRightRPM);
        topRightOutputLog.append(topRightOutput);
        
        // SmartDashboard.putNumber("Launcher/TopRight/SetpointRPM", targetTopRPM);
        // SmartDashboard.putNumber("Launcher/TopRight/MeasuredRPM", currentTopRightRPM);
        // SmartDashboard.putNumber("Launcher/TopRight/ErrorRPM", targetTopRPM - currentTopRightRPM);
        //SmartDashboard.putNumber("Launcher/TopRight/Output", topRightOutput);

        double middleOutput = launcher.getMiddleOutput();
        bottomTopSetpointLog.append(targetBottomRPM);
        bottomTopMeasuredLog.append(currentBottomTopRPM);
        bottomTopErrorLog.append(targetBottomRPM - currentBottomTopRPM);
        bottomTopOutputLog.append(middleOutput);
        
        // SmartDashboard.putNumber("Launcher/BottomTop/SetpointRPM", targetBottomRPM);
        // SmartDashboard.putNumber("Launcher/BottomTop/MeasuredRPM", currentBottomTopRPM);
        // SmartDashboard.putNumber("Launcher/BottomTop/ErrorRPM", targetBottomRPM - currentBottomTopRPM);
        //SmartDashboard.putNumber("Launcher/BottomTop/Output", middleOutput);

        double bottomBottomOutput = launcher.getBottomOutput();
        bottomBottomSetpointLog.append(targetBottomRPM/2);
        bottomBottomMeasuredLog.append(currentBottomBottomRPM);
        bottomBottomErrorLog.append(targetBottomRPM/2 - currentBottomBottomRPM);
        bottomBottomOutputLog.append(bottomBottomOutput);
        bottomActiveLog.append(readyToLaunch);
        
        // SmartDashboard.putNumber("Launcher/BottomBottom/SetpointRPM", targetBottomRPM/2);
        // SmartDashboard.putNumber("Launcher/BottomBottom/MeasuredRPM", currentBottomBottomRPM);
        // SmartDashboard.putNumber("Launcher/BottomBottom/ErrorRPM", targetBottomRPM/2 - currentBottomBottomRPM);
        // //SmartDashboard.putNumber("Launcher/BottomBottom/Output", bottomBottomOutput);
        // SmartDashboard.putBoolean("Launcher/BottomBottom/Active", readyToLaunch);
        
        // Debug logging
        // System.out.println("LauncherPID: TopLeft=" + String.format("%.2f", currentTopLeftRPM) + 
        //                   " TopRight=" + String.format("%.2f", currentTopRightRPM) +
        //                   " BottomTop=" + String.format("%.2f", currentBottomTopRPM) + 
        //                   " BottomBottom=" + String.format("%.2f", currentBottomBottomRPM) +
        //                   " | BottomActive=" + readyToLaunch + " (delay=" + String.format("%.2f", bottomLauncherDelay) + "s)");
        // System.out.println("LauncherPID:TargetRPM Top=" + String.format("%.2f", targetTopRPM) + 
        //                   " Bottom=" + String.format("%.2f", targetBottomRPM));
    }
    
    @Override
    public void end(boolean interrupted) {
        // Stop the launcher when command ends
        launcher.launch_stop();
        agitator.agitateMain_stop();
        if (interrupted) {
            System.out.println("LauncherPID: Command interrupted");
        } else {
            System.out.println("LauncherPID: Command finished at Top=" + String.format("%.0f", launcher.getTopRPM()) + 
                              " RPM, Bottom=" + String.format("%.0f", launcher.getBottomRPM()) + " RPM");
        }
    }
    
    // /**
    //  * Update PID constants for top launcher motors independently.
    //  */
    // public void updateTopPIDConstants(double kP, double kI, double kD, double kTolerance) {
    //     topLeftPIDController.setPID(kP, kI, kD);
    //     topRightPIDController.setPID(kP, kI, kD);
    //     topLeftPIDController.setTolerance(kTolerance);
    //     topRightPIDController.setTolerance(kTolerance);
    // }
    
    // /**
    //  * Update PID constants for bottom-top launcher motor independently.
    //  */
    // public void updateBottomTopPIDConstants(double kP, double kI, double kD, double kTolerance) {
    //     bottomTopPIDController.setPID(kP, kI, kD);
    //     bottomTopPIDController.setTolerance(kTolerance);
    // }
    
    // /**
    //  * Update PID constants for bottom-bottom launcher motor independently.
    //  */
    // public void updateBottomBottomPIDConstants(double kP, double kI, double kD, double kTolerance) {
    //     bottomBottomPIDController.setPID(kP, kI, kD);
    //     bottomBottomPIDController.setTolerance(kTolerance);
    // }
    
    // /**
    //  * Update PID constants for all bottom launcher motors together (deprecated - use separate methods).
    //  */
    // public void updateBottomPIDConstants(double kP, double kI, double kD, double kTolerance) {
    //     bottomTopPIDController.setPID(kP, kI, kD);
    //     bottomBottomPIDController.setPID(kP, kI, kD);
    //     bottomTopPIDController.setTolerance(kTolerance);
    //     bottomBottomPIDController.setTolerance(kTolerance);
    // }
    
    // /**
    //  * Update PID constants for all launcher motors.
    //  */
    // public void updatePIDConstants(double kP, double kI, double kD, double kTolerance) {
    //     topLeftPIDController.setPID(kP, kI, kD);
    //     topRightPIDController.setPID(kP, kI, kD);
    //     bottomTopPIDController.setPID(kP, kI, kD);
    //     bottomBottomPIDController.setPID(kP, kI, kD);
    //     topLeftPIDController.setTolerance(kTolerance);
    //     topRightPIDController.setTolerance(kTolerance);
    //     bottomTopPIDController.setTolerance(kTolerance);
    //     bottomBottomPIDController.setTolerance(kTolerance);
    // }
    
    
    @Override
    public boolean isFinished() {
        return false;
    }
    public double getTargetTopRPM(double targetTopRPM){
        if(LauncherConstants.kManualControl){
            double targetRPM = SmartDashboard.getNumber("GoalTopRPM", -1);
            return targetRPM;
        }
        else if (targetTopRPM ==123){
            return 3400;//test this
        }
        else if(targetTopRPM == 6767){
            return 4500;//test that
        }
        else if(!vision.hasTarget()){
            System.out.println("No Target");
            return LauncherConstants.kLauncherDefaultTopRPM;
        }
        else{
            double horizDistance = vision.getHorizDistance();
            double targetRPM = 3800;//LauncherConstants.SHOOTER_MAP.get(horizDistance).rpm();
            System.out.println("Target Distance: "+horizDistance+"  target RPM: "+targetRPM);
            return targetRPM;
        }
    }
    public double getTargetBottomRPM(double targetBottomRPM){
        if(LauncherConstants.kManualControl){
            double targetRPM = SmartDashboard.getNumber("GoalBottomRPM", -1);
            return targetRPM;
        }
        else if(targetBottomRPM ==0){
            return 0;
        }
        else{
            return LauncherConstants.kLauncherDefaultBottomRPM;
        }
    }
}
