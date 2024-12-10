package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;

public class Telemetry {
    private final double MaxSpeed;

    /**
     * Construct a telemetry object, with the specified max speed of the robot
     * 
     * @param maxSpeed Maximum speed in meters per second
     */
    public Telemetry(double maxSpeed) {
        MaxSpeed = maxSpeed;
        SignalLogger.start();
    }

    /* What to publish over networktables for telemetry */
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    /* Robot pose for field positioning */
    private final NetworkTable table = inst.getTable("Pose");
    private final DoubleArrayPublisher fieldPub = table.getDoubleArrayTopic("robotPose").publish();
    private final StringPublisher fieldTypePub = table.getStringTopic(".type").publish();

    /* Robot speeds for general checking */
    private final NetworkTable driveStats = inst.getTable("Drive");
    private final DoublePublisher velocityX = driveStats.getDoubleTopic("Velocity X").publish();
    private final DoublePublisher velocityY = driveStats.getDoubleTopic("Velocity Y").publish();
    private final DoublePublisher speed = driveStats.getDoubleTopic("Speed").publish();
    private final DoublePublisher odomFreq = driveStats.getDoubleTopic("Odometry Frequency").publish();

    /* Keep a reference of the last pose to calculate the speeds */
    private Pose2d m_lastPose = new Pose2d();
    private double lastTime = Utils.getCurrentTimeSeconds();

    private final double[] m_poseArray = new double[3];

    /** Accept the swerve drive state and telemeterize it to SmartDashboard and SignalLogger. */
    public void telemeterize(SwerveDriveState state) {
        /* Telemeterize the pose */
        Pose2d pose = state.Pose;
        m_poseArray[0] = pose.getX();
        m_poseArray[1] = pose.getY();
        m_poseArray[2] = pose.getRotation().getDegrees();

        fieldTypePub.set("Field2d");
        fieldPub.set(m_poseArray);

        /* Telemeterize the robot's general speeds */
        double currentTime = Utils.getCurrentTimeSeconds();
        double diffTime = currentTime - lastTime;
        lastTime = currentTime;

        Translation2d distanceDiff = pose.minus(m_lastPose).getTranslation();
        m_lastPose = pose;

        Translation2d velocities = distanceDiff.div(diffTime);

        speed.set(velocities.getNorm());
        velocityX.set(velocities.getX());
        velocityY.set(velocities.getY());
        odomFreq.set(1.0 / state.OdometryPeriod);

        /* Telemeterize the module's states */
        DogLog.log("Swerve/Target States", state.ModuleTargets);
        DogLog.log("Swerve/Actual States", state.ModuleStates);

        SignalLogger.writeDoubleArray("odometry", m_poseArray);
        SignalLogger.writeDouble("odom period", state.OdometryPeriod, "seconds");
    }
}
