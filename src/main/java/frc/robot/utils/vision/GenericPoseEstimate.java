package frc.robot.utils.vision;

import edu.wpi.first.math.geometry.Pose3d;

/* Provides a common interface for interacting with pose estimates from photonvision and limelight */
public class GenericPoseEstimate {
    public Pose3d estimatedPose;
    public int targetCount;
    public double ambiguity;
    public double timestampSeconds;
    public GenericFiducial[] fiducials;
}
