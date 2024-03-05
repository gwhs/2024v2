package frc.robot.subsystems.LimeVision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class ApriltagConstants {

    // switch between "POSE" OR "TAG"
    // pose uses robot posestimator
    // tag uses megatag values
    public static final String SWITCH = "POSE";

    public static final int[] APRILTAG_ROTATION = {120, 120, 180, 180, 270, 270, 0, 0, 60, 60, 300, 60, 180, 0, 120, 240};  

    // starts from 1-16
    public static final Pose2d[] TARGET_POSITION = {new Pose2d(14.88,0.65, new Rotation2d(ApriltagConstants.APRILTAG_ROTATION[0] - 180)),
                                                    new Pose2d(15.95,1.27, new Rotation2d(ApriltagConstants.APRILTAG_ROTATION[1] - 180)),
                                                    new Pose2d(15.17,5.55, new Rotation2d(ApriltagConstants.APRILTAG_ROTATION[2] - 180)),
                                                    new Pose2d(15.17,5.55, new Rotation2d(ApriltagConstants.APRILTAG_ROTATION[3] - 180)),
                                                    new Pose2d(14.70,7.30, new Rotation2d(ApriltagConstants.APRILTAG_ROTATION[4] - 180)),
                                                    new Pose2d(1.84,7.30, new Rotation2d(ApriltagConstants.APRILTAG_ROTATION[5] - 180)),
                                                    new Pose2d(1.29,5.54, new Rotation2d(ApriltagConstants.APRILTAG_ROTATION[6] - 180)),
                                                    new Pose2d(1.29,5.54, new Rotation2d(ApriltagConstants.APRILTAG_ROTATION[7] - 180)),
                                                    new Pose2d(0.59,1.27, new Rotation2d(ApriltagConstants.APRILTAG_ROTATION[8] - 180)),
                                                    new Pose2d(1.72,0.59, new Rotation2d(ApriltagConstants.APRILTAG_ROTATION[9] - 180)),
                                                    new Pose2d(12.42,2.86, new Rotation2d(ApriltagConstants.APRILTAG_ROTATION[10] - 180)),
                                                    new Pose2d(12.37,5.35, new Rotation2d(ApriltagConstants.APRILTAG_ROTATION[11] - 180)),
                                                    new Pose2d(10.22,4.10, new Rotation2d(ApriltagConstants.APRILTAG_ROTATION[12] - 180)),
                                                    new Pose2d(6.32,4.10, new Rotation2d(ApriltagConstants.APRILTAG_ROTATION[13] - 180)),
                                                    new Pose2d(4.12,5.32, new Rotation2d(ApriltagConstants.APRILTAG_ROTATION[14] - 180)),
                                                    new Pose2d(4.16,2.86, new Rotation2d(ApriltagConstants.APRILTAG_ROTATION[15] - 180))};
    
    public static final double[][] APRILTAG = {{15.08,0.24,1.35},
                                            {16.18,0.89,1.35},
                                            {16.58,4.98,1.45},
                                            {16.58,5.55,1.45},
                                            {14.70,8.23,1.35},
                                            {1.84,8.23,1.35},
                                            {-0.04,5.38,1.45},
                                            {-0.04,4.98,1.45},
                                            {0.36,0.88,1.35},
                                            {1.46,0.24,1.35},
                                            {11.90,3.71,1.32},
                                            {11.90,4.50,1.32},
                                            {11.22,4.10,1.32},
                                            {5.32,4.10,1.32},
                                            {4.64,4.50,1.32},
                                            {4.64,3.71,1.32}};  

    public static final class ForwardConstant {
        public static final double DISTANCE_TOLERANCE = 0.2;
        public static final double STEADY_STATE_TOLERANCE = 0.01;
    }

    public static final class SidewaysConstant {
        public static final double DISTANCE_TOLERANCE = 0.001;
        public static final double STEADY_STATE_TOLERANCE = 0.001;
    }

    public static final class RotationConstant {
        public static final double DISTANCE_TOLERANCE = 0.01;
        public static final double STEADY_STATE_TOLERANCE = 0.001;
    }

    public static final class TargetDistance {
        public static final double FORWARD_TARGET = 0.3; // in meters from tag
        public static final double SIDEWAYS_TARGET = 0; // leave at 0
        public static final double ROTATION_TARGET = 0; // leave at 0
    }


}