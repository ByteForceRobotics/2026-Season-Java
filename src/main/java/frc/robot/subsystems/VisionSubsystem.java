/*
 * MIT License
 *
 * Copyright (c) PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package frc.robot.subsystems;

import static frc.robot.Constants.CameraConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.List;
import java.util.ArrayList;
import java.util.function.Supplier;
import java.util.Deque;
import java.util.ArrayDeque;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator photonEstimator;
    private Matrix<N3, N1> curStdDevs;
    private final EstimateConsumer estConsumer;
    // buffer to hold recent candidate vision poses for stability check
    private final Deque<Pose2d> poseBuffer = new ArrayDeque<>();
    private final int poseBufferSize = 3;
    // stability thresholds (meters and radians)
    private double stabilityDistance = 0.5; // meters
    private double stabilityAngle = Math.toRadians(10); // radians
    // tuned std devs for forwarding measurements (modifiable via SmartDashboard)
    private Matrix<N3, N1> tunedSingleTagStd = VecBuilder.fill(0.5, 0.5, 0.3);
    private Matrix<N3, N1> tunedMultiTagStd = VecBuilder.fill(0.25, 0.25, 0.15);
    // snap/reset policy fields
    private int snapConfirmationCount = 0;
    private int snapConfirmationNeeded = 2;
    private final Supplier<Pose2d> currentPoseSupplier;


    /**
     * @param estConsumer Lamba that will accept a pose estimate and pass it to your desired {@link
     *     edu.wpi.first.math.estimator.SwerveDrivePoseEstimator}
     */
    public VisionSubsystem(EstimateConsumer estConsumer, Supplier<Pose2d> currentPoseSupplier) {
        this.estConsumer = estConsumer;
        this.currentPoseSupplier = currentPoseSupplier;
    camera = new PhotonCamera(kCameraName);
    // Use the same strategy as our other Camera helper: prefer multi-tag PNP on coprocessor when available
    photonEstimator = new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, kRobotToCam);
        // default to single-tag std devs until we calculate better values
        curStdDevs = kSingleTagStdDevs;
        // Dashboard defaults for tuning and snap policy. Only write defaults if keys are missing so we don't
        // overwrite values you set in Shuffleboard/SmartDashboard between deploys.
        if (!SmartDashboard.containsKey("Vision/StableDistance"))
            SmartDashboard.putNumber("Vision/StableDistance", stabilityDistance);
        if (!SmartDashboard.containsKey("Vision/StableAngleDeg"))
            SmartDashboard.putNumber("Vision/StableAngleDeg", Math.toDegrees(stabilityAngle));
        if (!SmartDashboard.containsKey("Vision/SingleStdX"))
            SmartDashboard.putNumber("Vision/SingleStdX", 0.5);
        if (!SmartDashboard.containsKey("Vision/SingleStdY"))
            SmartDashboard.putNumber("Vision/SingleStdY", 0.5);
        if (!SmartDashboard.containsKey("Vision/SingleStdTheta"))
            SmartDashboard.putNumber("Vision/SingleStdTheta", 0.3);
        if (!SmartDashboard.containsKey("Vision/MultiStdX"))
            SmartDashboard.putNumber("Vision/MultiStdX", 0.25);
        if (!SmartDashboard.containsKey("Vision/MultiStdY"))
            SmartDashboard.putNumber("Vision/MultiStdY", 0.25);
        if (!SmartDashboard.containsKey("Vision/MultiStdTheta"))
            SmartDashboard.putNumber("Vision/MultiStdTheta", 0.15);
        if (!SmartDashboard.containsKey("Vision/SnapEnabled"))
            SmartDashboard.putBoolean("Vision/SnapEnabled", false);
        if (!SmartDashboard.containsKey("Vision/SnapStdThresholdX"))
            SmartDashboard.putNumber("Vision/SnapStdThresholdX", 0.3);
        if (!SmartDashboard.containsKey("Vision/SnapStdThresholdY"))
            SmartDashboard.putNumber("Vision/SnapStdThresholdY", 0.3);
        if (!SmartDashboard.containsKey("Vision/SnapStdThresholdTheta"))
            SmartDashboard.putNumber("Vision/SnapStdThresholdTheta", 0.2);
        if (!SmartDashboard.containsKey("Vision/SnapDistanceThreshold"))
            SmartDashboard.putNumber("Vision/SnapDistanceThreshold", 1.0);
        if (!SmartDashboard.containsKey("Vision/SnapConsecutive"))
            SmartDashboard.putNumber("Vision/SnapConsecutive", 2);
    }

    public void periodic() {
        // Pull live tuning values from SmartDashboard so we can adjust parameters without redeploying
        stabilityDistance = SmartDashboard.getNumber("Vision/StableDistance", stabilityDistance);
        double stableAngleDeg = SmartDashboard.getNumber("Vision/StableAngleDeg", Math.toDegrees(stabilityAngle));
        stabilityAngle = Math.toRadians(stableAngleDeg);
        // update tuned stds if dashboard values changed
        double sX = SmartDashboard.getNumber("Vision/SingleStdX", tunedSingleTagStd.get(0, 0));
        double sY = SmartDashboard.getNumber("Vision/SingleStdY", tunedSingleTagStd.get(1, 0));
        double sT = SmartDashboard.getNumber("Vision/SingleStdTheta", tunedSingleTagStd.get(2, 0));
        tunedSingleTagStd = VecBuilder.fill(sX, sY, sT);
        double mX = SmartDashboard.getNumber("Vision/MultiStdX", tunedMultiTagStd.get(0, 0));
        double mY = SmartDashboard.getNumber("Vision/MultiStdY", tunedMultiTagStd.get(1, 0));
        double mT = SmartDashboard.getNumber("Vision/MultiStdTheta", tunedMultiTagStd.get(2, 0));
        tunedMultiTagStd = VecBuilder.fill(mX, mY, mT);

        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        // Diagnostic: print camera connection and name
        try {
            System.out.println("[Vision] cameraName=" + camera.getName() + " isConnected=" + camera.isConnected());
        } catch (Throwable ignored) {
        }

        var result = camera.getLatestResult();
        if (result != null) {
            // More detailed diagnostics about the pipeline result
            try {
                System.out.println("[Vision] latestResult hasTargets=" + result.hasTargets() + " targetsCount=" + result.getTargets().size() + " bestId=" + (result.getBestTarget() != null ? result.getBestTarget().getFiducialId() : "none"));
                for (var t : result.getTargets()) {
                    System.out.println("[Vision] target: id=" + t.getFiducialId() + " bestCamToTarget=" + t.getBestCameraToTarget());
                }
            } catch (Throwable ignored) {
            }
            Pose2d candidatePose = null;
            double candidateTimestamp = Timer.getFPGATimestamp();
            Matrix<N3, N1> candidateStd = null;
            int visibleTags = 0;

            visionEst = photonEstimator.estimateCoprocMultiTagPose(result);
            if (visionEst.isEmpty()) {
                visionEst = photonEstimator.estimateLowestAmbiguityPose(result);
            }
            // If still empty, add per-target diagnostics to help debug why estimation fails
            if (visionEst.isEmpty()) {
                // Collect manual per-target computed robot poses so we can fallback if estimator fails
                var manualPoses = new ArrayList<Pose2d>();
                var ambiguities = new ArrayList<Double>();
                for (var tgt : result.getTargets()) {
                    int id = tgt.getFiducialId();
                    double ambiguity = Double.NaN;
                    try {
                        ambiguity = tgt.getPoseAmbiguity();
                    } catch (Throwable ignored) {
                    }
                    var maybeTagPose = photonEstimator.getFieldTags().getTagPose(id);
                    System.out.println("[Vision][DBG] target id=" + id + " ambiguity=" + ambiguity + " hasFieldPose=" + maybeTagPose.isPresent());
                    if (maybeTagPose.isPresent()) {
                        try {
                            var tagPose = maybeTagPose.get();
                            var camToTarget = tgt.getBestCameraToTarget();
                            // Compute camera pose in field: tagPose * inverse(camToTarget)
                            var camPoseField = tagPose.transformBy(camToTarget.inverse());
                            // Compute robot pose in field: cameraPose * inverse(robotToCam)
                            var robotPoseField = camPoseField.transformBy(kRobotToCam.inverse());
                            Pose2d manual = robotPoseField.toPose2d();
                            manualPoses.add(manual);
                            ambiguities.add(ambiguity);
                            System.out.println("[Vision][DBG] computed robotPose from target " + id + " = " + manual);
                        } catch (Throwable e) {
                            System.out.println("[Vision][DBG] error computing manual pose for tag " + id + ": " + e);
                        }
                    } else {
                        System.out.println("[Vision][DBG] no field pose for tag " + id + "; check aprilTagFieldLayout.json and tag IDs");
                    }
                }

                // If estimator failed but we have manual poses, create a conservative fallback measurement
                if (manualPoses.size() > 0) {
                    // Compute average translation and mean rotation (via unit-vector averaging)
                    double sumX = 0.0;
                    double sumY = 0.0;
                    double sumCos = 0.0;
                    double sumSin = 0.0;
                    for (var p : manualPoses) {
                        sumX += p.getX();
                        sumY += p.getY();
                        sumCos += p.getRotation().getCos();
                        sumSin += p.getRotation().getSin();
                    }
                    int n = manualPoses.size();
                    Pose2d averaged = new Pose2d(sumX / n, sumY / n, new Rotation2d(Math.atan2(sumSin / n, sumCos / n)));
                    // Compute a WPILib-style timestamp using result latency if available
                    double visionTimestamp = Timer.getFPGATimestamp();
                    try {
                        java.lang.reflect.Method m = result.getClass().getMethod("getLatencyMillis");
                        Object latencyObj = m.invoke(result);
                        if (latencyObj instanceof Number) {
                            double latencyMs = ((Number) latencyObj).doubleValue();
                            visionTimestamp = Timer.getFPGATimestamp() - (latencyMs / 1000.0);
                            System.out.println("[Vision] fallback latencyMs=" + latencyMs + " computedTimestamp=" + visionTimestamp);
                        }
                    } catch (Throwable ignored) {
                    }

                    // Use a conservative std dev (larger than single-tag) so estimator treats this as low-confidence
                    var fallbackStd = kSingleTagStdDevs.times(5);
                    System.out.println("[Vision] prepared fallback manualPose=" + averaged + " timestamp=" + visionTimestamp + " stdDevs=" + fallbackStd);
                    // set candidate to be considered by stability filter
                    candidatePose = averaged;
                    candidateTimestamp = visionTimestamp;
                    candidateStd = fallbackStd;
                    visibleTags = manualPoses.size();
                }
            }
            updateEstimationStdDevs(visionEst, result.getTargets());
            // Basic runtime debugging: detection count, estimator result, and timestamps
            SmartDashboard.putNumber("Vision/Detections", result.getTargets().size());
            SmartDashboard.putString("Vision/LastEstimate", visionEst.toString());
            SmartDashboard.putNumber("Vision/FPGA_Time", Timer.getFPGATimestamp());
            visionEst.ifPresent(est -> {
                SmartDashboard.putNumber("Vision/Estimate_Timestamp", est.timestampSeconds);
            });
            System.out.println("[Vision] detections=" + result.getTargets().size() + " estimatePresent=" + visionEst.isPresent() + " fpgaTime=" + Timer.getFPGATimestamp());
            if (visionEst.isPresent()) {
                var est = visionEst.get();
                // Change our trust in the measurement based on the tags we can see
                var estStdDevs = getEstimationStdDevs();

                // Log the estimated pose before forwarding
                System.out.println("[Vision] estimatedPose=" + est.estimatedPose.toPose2d() + " estTimestamp=" + est.timestampSeconds + " stdDevs=" + estStdDevs);

                // Compute a WPILib-friendly timestamp for the vision measurement.
                double visionTimestamp = est.timestampSeconds; // default
                try {
                    var res = result; // effectively final for reflection
                    java.lang.reflect.Method m = res.getClass().getMethod("getLatencyMillis");
                    Object latencyObj = m.invoke(res);
                    if (latencyObj instanceof Number) {
                        double latencyMs = ((Number) latencyObj).doubleValue();
                        visionTimestamp = Timer.getFPGATimestamp() - (latencyMs / 1000.0);
                        System.out.println("[Vision] latencyMs=" + latencyMs + " computedTimestamp=" + visionTimestamp);
                    }
                } catch (Throwable ignored) {
                }

                // Immediately forward estimator-derived poses (faster corrections). Snap/reset policy applies only when
                // enabled and measurements are deemed high-confidence and the robot appears far from the current pose.
                Pose2d visionPose = est.estimatedPose.toPose2d();
                var usedStd = estStdDevs;

                // Read snap policy / thresholds from dashboard
                boolean snapEnabled = SmartDashboard.getBoolean("Vision/SnapEnabled", false);
                double snapStdX = SmartDashboard.getNumber("Vision/SnapStdThresholdX", 0.3);
                double snapStdY = SmartDashboard.getNumber("Vision/SnapStdThresholdY", 0.3);
                double snapStdTheta = SmartDashboard.getNumber("Vision/SnapStdThresholdTheta", 0.2);
                double snapDistance = SmartDashboard.getNumber("Vision/SnapDistanceThreshold", 1.0);
                int snapNeeded = (int) SmartDashboard.getNumber("Vision/SnapConsecutive", 2);

                double stdX = Math.abs(usedStd.get(0, 0));
                double stdY = Math.abs(usedStd.get(1, 0));
                double stdTheta = Math.abs(usedStd.get(2, 0));

                boolean highConfidence = stdX <= snapStdX && stdY <= snapStdY && stdTheta <= snapStdTheta;
                boolean shouldForceReset = false;
                if (snapEnabled && highConfidence) {
                    try {
                        Pose2d cur = currentPoseSupplier.get();
                        double dist = cur.getTranslation().getDistance(visionPose.getTranslation());
                        if (dist > snapDistance) {
                            snapConfirmationCount++;
                            System.out.println("[Vision] snap candidate dist=" + dist + " confirmation=" + snapConfirmationCount + "/" + snapNeeded);
                            if (snapConfirmationCount >= snapNeeded) {
                                shouldForceReset = true;
                                snapConfirmationCount = 0;
                            }
                        } else {
                            snapConfirmationCount = 0;
                        }
                    } catch (Throwable e) {
                        // If anything goes wrong, don't force reset
                        snapConfirmationCount = 0;
                    }
                } else {
                    // Not a snap candidate, reset counter
                    snapConfirmationCount = 0;
                }

                // Forward immediately (fast correction) but with forceReset only when snap policy fires
                estConsumer.accept(visionPose, visionTimestamp, usedStd, shouldForceReset);
                curStdDevs = usedStd;
                // Do not add estimator-derived poses to the stability buffer
            }

            // After collecting candidatePose (either estimator or fallback), run stability filter
            if (candidatePose != null) {
                poseBuffer.addLast(candidatePose);
                if (poseBuffer.size() > poseBufferSize) poseBuffer.removeFirst();

                if (poseBuffer.size() >= 2) {
                    double sumX = 0, sumY = 0, sumCos = 0, sumSin = 0;
                    for (var p : poseBuffer) {
                        sumX += p.getX();
                        sumY += p.getY();
                        sumCos += p.getRotation().getCos();
                        sumSin += p.getRotation().getSin();
                    }
                    int n = poseBuffer.size();
                    Pose2d avg = new Pose2d(sumX / n, sumY / n, new Rotation2d(Math.atan2(sumSin / n, sumCos / n)));
                    boolean stable = true;
                    for (var p : poseBuffer) {
                        if (p.getTranslation().getDistance(avg.getTranslation()) > stabilityDistance) {
                            stable = false;
                            break;
                        }
                        if (Math.abs(p.getRotation().minus(avg.getRotation()).getRadians()) > stabilityAngle) {
                            stable = false;
                            break;
                        }
                    }
                    if (stable) {
                        var usedStd = candidateStd != null ? candidateStd : (visibleTags > 1 ? tunedMultiTagStd : tunedSingleTagStd);
                        System.out.println("[Vision] stable average pose=" + avg + " forwarding with std=" + usedStd);
                        estConsumer.accept(avg, candidateTimestamp, usedStd, false);
                        curStdDevs = usedStd;
                        poseBuffer.clear();
                    } else {
                        System.out.println("[Vision] candidate not yet stable; bufferSize=" + poseBuffer.size());
                    }
                }
            }
        } else {
            // No result available; update dashboard to show zero detections
            SmartDashboard.putNumber("Vision/Detections", 0);
            SmartDashboard.putString("Vision/LastEstimate", "Optional.empty");
            SmartDashboard.putNumber("Vision/FPGA_Time", Timer.getFPGATimestamp());
            System.out.println("[Vision] detections=0 estimatePresent=false fpgaTime=" + Timer.getFPGATimestamp());
        }
    }

    /**
     * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
     * deviations based on number of tags, estimation strategy, and distance from the tags.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     * @param targets All targets in this camera frame
     */
    private void updateEstimationStdDevs(
            Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            curStdDevs = kSingleTagStdDevs;

        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = kSingleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an average-distance metric
            for (var tgt : targets) {
                var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty()) continue;
                numTags++;
                avgDist +=
                        tagPose
                                .get()
                                .toPose2d()
                                .getTranslation()
                                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                curStdDevs = kSingleTagStdDevs;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1) estStdDevs = kMultiTagStdDevs;
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                curStdDevs = estStdDevs;
            }
        }
    }

    /**
     * Returns the latest standard deviations of the estimated pose from {@link
     * #getEstimatedGlobalPose()}, for use with {@link
     * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}. This should
     * only be used when there are targets visible.
     */
    public Matrix<N3, N1> getEstimationStdDevs() {
        return curStdDevs;
    }
       

    @FunctionalInterface
    public static interface EstimateConsumer {
        /**
         * @param pose estimated Pose2d
         * @param timestamp timestamp in seconds (WPILib time)
         * @param estimationStdDevs measurement std devs
         * @param forceReset if true, consumer should reset odometry to pose instead of addVisionMeasurement
         */
        public void accept(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs, boolean forceReset);
    }
}