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
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import static frc.robot.Constants.VisionConstants.PhotonVision.kLocalizationStrategy;
import static frc.robot.Constants.VisionConstants.PhotonVision.kFallbackStrategy;
import static frc.robot.Constants.VisionConstants.PhotonVision.kSingleTagDefaultStdDevs;
import static frc.robot.Constants.VisionConstants.PhotonVision.kMultiTagDefaultStdDevs;

public class FusedPoseEstimator extends SwerveDrivePoseEstimator {
    
    private ArrayList<PhotonSource> PhotonSources;
    
    private class PhotonSource {

        private final Supplier<List<PhotonPipelineResult>> rawSource;
        private final PhotonPoseEstimator estimator;
        public Matrix<N3,N1> curStdDevs;

        //Cameras are represented by lambda functions so as to not imply ownership
        public PhotonSource (PhotonCamera cam,Transform3d offset){
            this.rawSource = () -> cam.getAllUnreadResults();
            this.estimator = new PhotonPoseEstimator(AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField), kLocalizationStrategy, offset);
            this.estimator.setMultiTagFallbackStrategy(kFallbackStrategy);
        }

        public Optional<EstimatedRobotPose> getEstimatedPose() {
            Optional<EstimatedRobotPose> visionEst = Optional.empty();
            for (var res : rawSource.get()) {
                visionEst = estimator.update(res);
                updateEstimationStdDevs(visionEst,res.getTargets());
            }
            return visionEst;
        }

        /* Dynamically calculates standard deviations for pose estimation */
        private void updateEstimationStdDevs(
            Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
            if (estimatedPose.isEmpty()) {
                // No pose input. Default to single-tag std devs
                curStdDevs = kSingleTagDefaultStdDevs;
            } else {
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
                            .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
                }

                if (numTags == 0) {
                    // No tags visible. Default to single-tag std devs
                    curStdDevs = kSingleTagDefaultStdDevs;
                } else {
                    // One or more tags visible, run the full heuristic.
                    avgDist /= numTags;
                    // Decrease std devs if multiple targets are visible
                    if (numTags > 1) estStdDevs = kMultiTagDefaultStdDevs;
                    // Increase std devs based on (average) distance
                    if (numTags == 1 && avgDist > 4)
                        estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                    else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                        curStdDevs = estStdDevs;
                }
            }
        }
    }

    public FusedPoseEstimator(SwerveDriveKinematics kinematics, Rotation2d gyroAngle, SwerveModulePosition[] modulePositions, Pose2d initialPoseMeters){
        super(kinematics,gyroAngle,modulePositions,initialPoseMeters);
    }

    public FusedPoseEstimator(SwerveDriveKinematics kinematics, Rotation2d gyroAngle, SwerveModulePosition[] modulePositions, Pose2d initialPoseMeters, Matrix<N3,N1> stateStdDevs, Matrix<N3,N1> visionMeasurementStdDevs){
        super(kinematics,gyroAngle,modulePositions,initialPoseMeters,stateStdDevs,visionMeasurementStdDevs);
    }

    public void registerPhotonCam(PhotonCamera cam, Transform3d offset){
        PhotonSources.add(new PhotonSource(cam,offset));
    }

    public void updateWithVision() {
        for (PhotonSource photon : PhotonSources) {
            Optional<EstimatedRobotPose> pose = photon.getEstimatedPose();
            if (pose.isPresent()){
                super.addVisionMeasurement(
                    pose.get().estimatedPose.toPose2d(),
                    pose.get().timestampSeconds,
                    photon.curStdDevs
                );
            }
        }
    }
}
