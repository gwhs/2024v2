package frc.robot.subsystems.LimeVision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;


public class ApriltagController extends SubsystemBase {

    private final SwerveSubsystem swerve;
    private final LimeLightSub limeLightSub;

    // forward PID constants
    private final double kPX = 1; //0.05
    private final double kDX = 0;
    private final double kIX = 0;

    // sideways PID constants
    private final double kPThetaTx = 0.02;
    private final double kDThetaTx = 0;
    private final double kIThetaTx = 0;    

    // rotation PID constants
    private final double kPTheta = 0.03;
    private final double kDTheta = 0;
    private final double kITheta = 0;

    private PIDController PIDForward = new PIDController(kPX, kIX, kDX);
    private PIDController PIDRotation = new PIDController(kPTheta, kITheta, kDTheta);
    private PIDController PIDSideways = new PIDController(kPThetaTx, kIThetaTx, kDThetaTx);

    private double forwardOutput;
    private double sidewaysOutput;
    private double rotationOutput;

    public ApriltagController(SwerveSubsystem swerve, LimeLightSub limeLightSub) {
        this.swerve = swerve;
        this.limeLightSub = limeLightSub;
    }

    // front and back
    public double getDistanceForward() {
        if (limeLightSub.getID() > 0) {
            return Math.abs(ApriltagConstants.APRILTAG[limeLightSub.getID()][0] - swerve.getPose().getX());
        }
        return Double.POSITIVE_INFINITY;
    }
    public double updatePIDForward() {
        forwardOutput = PIDForward.calculate(getDistanceForward()); 
        return forwardOutput;
    }
    public double getErrorForward() {
        return forwardOutput;
    }

    // testing using botpose distance
    public double getBotPoseDistance() {
        if (limeLightSub.getID() > 0) {
        return Math.sqrt(Math.pow(ApriltagConstants.APRILTAG[limeLightSub.getID()][0] - limeLightSub.getBotPose()[0],2) + 
                         Math.pow(ApriltagConstants.APRILTAG[limeLightSub.getID()][1] - limeLightSub.getBotPose()[1],2));
        }
        return 0;
    }
    public double updatePIDForwardBotPose() {
        forwardOutput = PIDForward.calculate(getBotPoseDistance()); 
        return forwardOutput;
    }

    // left and right
    public double getDistanceSideways() {
        if (limeLightSub.getID() > 0) {
            return ApriltagConstants.APRILTAG[limeLightSub.getID()][1] - swerve.getPose().getY();
        }
        return Double.POSITIVE_INFINITY;
    }
    public double updatePIDSideways() {
        sidewaysOutput = PIDSideways.calculate(limeLightSub.getTx());
        return sidewaysOutput;
    }
    public double getErrorSideways() {
        return sidewaysOutput;
    }

    // rotation yaw
    public double updatePIDRotation() {
        rotationOutput = PIDRotation.calculate(limeLightSub.getTx());
        return rotationOutput;
    }
    public double getErrorRotation() {
        return rotationOutput;
    }
    public double getApriltagHeading() {
        if (limeLightSub.getID() > 0) {
            return ApriltagConstants.APRILTAG_ROTATION[limeLightSub.getID()] + 180;
        }
        return swerve.getPose().getRotation().getDegrees();
    }

    //robot pose
    public double getHeading() {
        return swerve.getPose().getRotation().getDegrees();
    }

  // setsPoint PID
    public void setPoint(double target, String orientation) {
        if (orientation.toLowerCase().equals("forward")) {
            PIDForward.setSetpoint(target);
        } else if (orientation.toLowerCase().equals("sideways")) {
            PIDSideways.setSetpoint(target);
        } else if (orientation.toLowerCase().equals("rotation")) {
            PIDRotation.setSetpoint(target);
        }
    }
    
    //sets tolerances
    public void setTolerance(String PIDType)  {
        if (PIDType.toLowerCase().equals("forward")) {
            PIDForward.setTolerance(ApriltagConstants.ForwardConstant.DISTANCE_TOLERANCE, ApriltagConstants.ForwardConstant.STEADY_STATE_TOLERANCE);
        } else if (PIDType.toLowerCase().equals("sideways")) {
            PIDSideways.setTolerance(ApriltagConstants.SidewaysConstant.DISTANCE_TOLERANCE, ApriltagConstants.SidewaysConstant.STEADY_STATE_TOLERANCE);
        }
        else if (PIDType.toLowerCase().equals("rotation")) {
            PIDSideways.setTolerance(ApriltagConstants.RotationConstant.DISTANCE_TOLERANCE, ApriltagConstants.RotationConstant.STEADY_STATE_TOLERANCE);
        }
    }

    // isFinished
    public boolean atSetpoint(String PIDType) {
        if (PIDType.toLowerCase().equals("forward")) {
            return PIDForward.atSetpoint();
        } else if (PIDType.toLowerCase().equals("sideways")) {
            return PIDSideways.atSetpoint();
        } else if (PIDType.toLowerCase().equals("rotation")) {
            return PIDSideways.atSetpoint();
        }
        return false;
    }

    public double getDerivative(String PIDType) {
        double velocityError = Double.POSITIVE_INFINITY;
        if (PIDType.toLowerCase().equals("forward")) {
            velocityError = PIDForward.getVelocityError();
        } else if (PIDType.toLowerCase().equals("sideways")) {
            velocityError = PIDSideways.getVelocityError();
        } else if (PIDType.toLowerCase().equals("rotation")) {
            velocityError = PIDRotation.getVelocityError();
        }
        return velocityError;
    }
}