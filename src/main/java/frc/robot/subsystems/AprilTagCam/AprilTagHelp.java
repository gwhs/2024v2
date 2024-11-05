package frc.robot.subsystems.AprilTagCam;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class AprilTagHelp {

    Matrix<N3, N1> sd;
    Pose2d pos;
    double timestamp;

    public AprilTagHelp(Matrix<N3, N1> sd, Pose2d pos, double timestamp) {
        this.sd = sd;
        this.pos = pos;
        this.timestamp = timestamp;
    }

}
