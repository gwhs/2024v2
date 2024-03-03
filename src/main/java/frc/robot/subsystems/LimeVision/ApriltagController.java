package frc.robot.subsystems.LimeVision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;


public class ApriltagController extends SubsystemBase {

    private final SwerveSubsystem swerve;
    private final LimeLightSub limeLightSub;

    // forward PID constants
    private final double kPX = 0.8; //0.05
    private final double kDX = 0;
    private final double kIX = 0;

    // sideways PID constants
    private final double kPThetaTx = 0.02; //0.02
    private final double kDThetaTx = 0;
    private final double kIThetaTx = 0;    

    // rotation PID constants
    private final double kPTheta = 0.08; //0.08
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

    // forward and backwards
    // if setData works, remove otherwise
    // ====
    public double getDistanceForward() {
        if (limeLightSub.getID() > 0) {
            limeLightSub.setData();
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

    // sideways (left and right)
    // if setData works, remove otherwise
    // ====
    public double getDistanceSideways() {
        if (limeLightSub.getID() > 0) {
            limeLightSub.setData();
            return ApriltagConstants.APRILTAG[limeLightSub.getID()][1] - swerve.getPose().getY();
        }
        return Double.POSITIVE_INFINITY;
    }
    // Use PIDSideways.calculate(limeLightSub.getTx()) if other does not work
    // ====
    public double updatePIDSideways() {
        if (ApriltagConstants.SWITCH.equals("TAG")) {
            sidewaysOutput = PIDSideways.calculate(limeLightSub.getTx());
        } else if (ApriltagConstants.SWITCH.equals("POSE")) {
            sidewaysOutput = PIDSideways.calculate(getDistanceSideways());
        }
        return sidewaysOutput;
    }
    public double getErrorSideways() {
        return sidewaysOutput;
    }

    // rotation yaw
    // used for aligning speaker
    public double updatePIDRotation() {
        rotationOutput = PIDRotation.calculate(limeLightSub.getTx());
        return rotationOutput;
    }
    public double getErrorRotation() {
        return rotationOutput;
    }

    // returns the degree that is parallel to the tag (field relative)
    // uses botPose to find angle difference
    // uses switch bc don't know if math is the same using robot heading, should be the same if setData works
    // ====
    public double getApriltagHeading() {
        if (limeLightSub.getID() > 0 && ApriltagConstants.SWITCH.equals("POSE")) {
            limeLightSub.setData();
            double targetHeading = ApriltagConstants.APRILTAG_ROTATION[limeLightSub.getID()] - 180;
            return swerve.getPose().getRotation().getDegrees() - targetHeading;

        } else if (limeLightSub.getID() > 0 && ApriltagConstants.SWITCH.equals("TAG")) {
            double targetHeading = ApriltagConstants.APRILTAG_ROTATION[limeLightSub.getID()] - 180;
            return limeLightSub.getBlueBotPose()[5] - targetHeading;
        }
        return 0;
    }
    // returns error from target parallel heading
    public double updatePIDRotationParallel() {
        rotationOutput = PIDRotation.calculate(getApriltagHeading());
        return rotationOutput;
    }

    // setsPoint PID
    public void setPoint(double target, String PIDType) {
        if (PIDType.toLowerCase().equals("forward")) {
            PIDForward.setSetpoint(target);
        } else if (PIDType.toLowerCase().equals("sideways")) {
            PIDSideways.setSetpoint(target);
        } else if (PIDType.toLowerCase().equals("rotation")) {
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
        } else {
            return PIDRotation.atSetpoint();
        }
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

    // used for driveToPose, drives to a fixed target position
    public Pose2d getTargetPose() {
        if (limeLightSub.getID() > 0) {
            return ApriltagConstants.TARGET_POSITION[limeLightSub.getID()];
        }
        return swerve.getPose();
    }

    public double getHeading() {
        return swerve.getPose().getRotation().getDegrees();
    }

    // Rotation matrix for coordinate transformation
    public Translation2d fieldRelative(double x, double y) {
        if (ApriltagConstants.SWITCH.equals("POSE")) {
            double newX = x * Math.cos(getHeading()) - y * Math.sin(getHeading());
            double newY = x * Math.sin(getHeading()) + y * Math.cos(getHeading());
            return new Translation2d(newX,newY);
        }
        return new Translation2d(forwardOutput, sidewaysOutput);
    }
}