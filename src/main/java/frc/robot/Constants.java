// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.geometry.Rotation3d;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class VisionConstants {
        public static enum CamConfig {
            CAM0("camera0",0,0,0,0,0,0);

            public final String name;
            public final double x;
            public final double y;
            public final double z;
            public final double roll;
            public final double pitch;
            public final double yaw;
            public final Transform3d offset;
            private CamConfig(String name, double x, double y, double z, double roll, double pitch, double yaw){
                this.name = name;
                this.x = x;
                this.y = y;
                this.z = z;
                this.roll = roll;
                this.pitch = pitch;
                this.yaw = yaw;
                this.offset = new Transform3d(x,y,z,new Rotation3d(roll,pitch,yaw));
            }
        }
        public static class PhotonVision {
            public static final PoseStrategy kLocalizationStrategy = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;
            public static final PoseStrategy kFallbackStrategy = PoseStrategy.LOWEST_AMBIGUITY;
            public static final Matrix<N3, N1> kSingleTagDefaultStdDevs = VecBuilder.fill(4, 4, 8);
            public static final Matrix<N3, N1> kMultiTagDefaultStdDevs = VecBuilder.fill(0.5, 0.5, 1);
        }
    }
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}
