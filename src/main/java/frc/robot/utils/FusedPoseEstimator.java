package frc.robot.utils;

import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class FusedPoseEstimator extends SwerveDrivePoseEstimator {
    
    private ArrayList<GenericSource> Sources;
    
    public FusedPoseEstimator(SwerveDriveKinematics kinematics, Rotation2d gyroAngle, SwerveModulePosition[] modulePositions, Pose2d initialPoseMeters){
        super(kinematics,gyroAngle,modulePositions,initialPoseMeters);
    }

    public FusedPoseEstimator(SwerveDriveKinematics kinematics, Rotation2d gyroAngle, SwerveModulePosition[] modulePositions, Pose2d initialPoseMeters, Matrix<N3,N1> stateStdDevs, Matrix<N3,N1> visionMeasurementStdDevs){
        super(kinematics,gyroAngle,modulePositions,initialPoseMeters,stateStdDevs,visionMeasurementStdDevs);
    }

    public void registerPhotonCam(PhotonCamera cam, Transform3d offset){
        Sources.add(new PhotonSource(cam,offset));
    }

    /** Processes measurements from all registered vision sources */
    public void processVision() {
        for (GenericSource source : Sources) {
            Optional<GenericSource.GenericPoseEstimate> pose = source.getPoseEstimate();
            if (pose.isPresent()){
                super.addVisionMeasurement(
                    pose.get().estimatedPose(),
                    pose.get().timestampSeconds(),
                    pose.get().stdDevs()
                );
            }
        }
    }
}
