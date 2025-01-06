package frc.robot.utils;

/* A generic interface for handling vision sources */

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * A generic interface for handling vision sources
 */
public interface GenericSource {

    public static record GenericPoseEstimate(Pose2d estimatedPose, double timestampSeconds, Matrix<N3,N1> stdDevs) {}
    
    /* Returns an estimated pose */
    public abstract Optional<GenericPoseEstimate> getPoseEstimate();

}