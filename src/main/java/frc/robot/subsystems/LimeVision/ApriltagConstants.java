package frc.robot.subsystems.LimeVision;

public class ApriltagConstants {

    public static final class ForwardConstant {
        public static final double STEADY_STATE_TOLERANCE = 0.01;
        public static final double DISTANCE_TOLERANCE = 0.1;
    }

    public static final class SidewaysConstant {
        public static final double STEADY_STATE_TOLERANCE = 0.001;
        public static final double DISTANCE_TOLERANCE = 0.001;
    }

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

public static final int[] APRILTAG_ROTATION = {120, 120, 180, 180, 270, 270, 0, 0, 60, 60, 300, 60, 180, 0, 120, 240};
    
    
}
