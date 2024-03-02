package frc.robot.subsystems.LimeVision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class ApriltagConstants {

    // find trajectory values

    public static final int[] APRILTAG_ROTATION = {120, 120, 180, 180, 270, 270, 0, 0, 60, 60, 300, 60, 180, 0, 120, 240};  

    public static final Pose2d[] TARGET_POSITION = {new Pose2d(0,0, new Rotation2d(0)),
                                                    new Pose2d(0,0, new Rotation2d(0)),
                                                    new Pose2d(0,0, new Rotation2d(0)),
                                                    new Pose2d(0,0, new Rotation2d(0)),
                                                    new Pose2d(0,0, new Rotation2d(0)),
                                                    new Pose2d(0,0, new Rotation2d(0)),
                                                    new Pose2d(1.29,5.54, new Rotation2d(ApriltagConstants.APRILTAG_ROTATION[6] - 180)),
                                                    new Pose2d(1.29,5.54, new Rotation2d(ApriltagConstants.APRILTAG_ROTATION[7] - 180)),
                                                    new Pose2d(0,0, new Rotation2d(0)),
                                                    new Pose2d(0,0, new Rotation2d(0)),
                                                    new Pose2d(0,0, new Rotation2d(0)),
                                                    new Pose2d(0,0, new Rotation2d(0)),
                                                    new Pose2d(0,0, new Rotation2d(0)),
                                                    new Pose2d(6.32,4.08, new Rotation2d(ApriltagConstants.APRILTAG_ROTATION[13] - 180)),
                                                    new Pose2d(4.17,5.13, new Rotation2d(ApriltagConstants.APRILTAG_ROTATION[14] - 180)),
                                                    new Pose2d(4.19,2.94, new Rotation2d(ApriltagConstants.APRILTAG_ROTATION[15] - 180))};
    
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
        public static final double DISTANCE_TOLERANCE = 0.1;
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
