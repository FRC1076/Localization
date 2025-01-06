package frc.robot.utils;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import static frc.robot.Constants.VisionConstants.PhotonVision.kLocalizationStrategy;
import static frc.robot.Constants.VisionConstants.PhotonVision.kFallbackStrategy;
import static frc.robot.Constants.VisionConstants.PhotonVision.kSingleTagDefaultStdDevs;
import static frc.robot.Constants.VisionConstants.PhotonVision.kMultiTagDefaultStdDevs;

public class PhotonSource implements GenericSource {
    
    private final PhotonCamera cam;
    private final PhotonPoseEstimator estimator;
    
    public PhotonSource(PhotonCamera cam, Transform3d offset) {
        this.cam = cam;
        this.estimator = new PhotonPoseEstimator(AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField), kLocalizationStrategy, offset);
        this.estimator.setMultiTagFallbackStrategy(kFallbackStrategy);
    }

    @Override
    public Optional<GenericPoseEstimate> getPoseEstimate() {
        Optional<GenericPoseEstimate> visionEst = Optional.empty();
        List<PhotonPipelineResult> results = cam.getAllUnreadResults();
        for (var res : results) {
            Optional<EstimatedRobotPose> rawEst = estimator.update(res);
            if (rawEst.isPresent()) {
                Pose2d pose = rawEst.get().estimatedPose.toPose2d();
                visionEst = Optional.of(new GenericPoseEstimate(
                    pose, 
                    rawEst.get().timestampSeconds, 
                    calculateStdDevs(pose, rawEst.get().targetsUsed)
                ));
            }
        }
        return visionEst;
    }

    private Matrix<N3,N1> calculateStdDevs(Pose2d pose, List<PhotonTrackedTarget> targets) {
        // Pose present. Start running Heuristic
        var estStdDevs = kSingleTagDefaultStdDevs;
        int numTags = 0;
        double avgDist = 0;
        // Precalculation - see how many tags we found, and calculate an average-distance metric
        for (var tgt : targets) {
            var tagPose = estimator.getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty()) continue;
            numTags++;
            avgDist +=
                tagPose
                    .get()
                    .toPose2d()
                    .getTranslation()
                    .getDistance(pose.getTranslation());
        }
        if (numTags != 0) {
            // One or more tags visible, run the full heuristic.
            avgDist /= numTags;
            // Decrease std devs if multiple targets are visible
            if (numTags > 1) estStdDevs = kMultiTagDefaultStdDevs;
            // Increase std devs based on (average) distance
            if (numTags == 1 && avgDist > 4)
                estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
            else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
        }
        return estStdDevs;
    }


}

