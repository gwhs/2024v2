package frc.robot.subsystems.LimeVision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;


public class ApriltagController extends SubsystemBase {

    private final SwerveSubsystem swerve;
    private final LimeLightSub limeLightSub;

    // forward PID constants
    private final double kPX = 0.3;
    private final double kDX = 0.1;
    private final double kIX = 0;

    // sideways PID constants
    // private final double kPY = 1;
    // private final double kDY = 0;
    // private final double kIY = 0;

    // rotation PID constants
    private final double kPTheta = 0.4;
    private final double kDTheta = 0;
    private final double kITheta = 0;

    // sideways PID constants
    private final double kPThetaTx = 0.02;
    private final double kDThetaTx = 0;
    private final double kIThetaTx = 0;
    
    private PIDController PIDForward = new PIDController(kPX, kIX, kDX);
    // private PIDController PIDSideways = new PIDController(kPY, kIY, kDY);
    private PIDController PIDRotation = new PIDController(kPTheta, kITheta, kDTheta);
    private PIDController PIDSideways = new PIDController(kPThetaTx, kIThetaTx, kDThetaTx);

    private double[][] apriltag = {{15.08,0.24,1.35},
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
            return apriltag[limeLightSub.getID()][0] - swerve.getPose().getX();
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

    // left and right
    public double getDistanceSideways() {
        if (limeLightSub.getID() > 0) {
            return apriltag[limeLightSub.getID()][1] - swerve.getPose().getY();
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
            return apriltag[limeLightSub.getID()][2] + 180;
        }
        return swerve.getPose().getRotation().getDegrees();
    }
    public double getRobotHeading() {
        return swerve.getPose().getRotation().getDegrees();
    }
    

    // public double updatePIDThetaTx() {
    //     return PIDVisionThetaTx.calculate(limeLightSub.getSmoothTheta());
    // }

    // public double getPIDGThetaTX() {
    //     return PIDVisionThetaTx.calculate(limeLightSub.getTx());
    // }


  // setsPoint PID
    public void setPoint(double target, String orientation) {
        if (orientation.toLowerCase().equals("forward")) {
            PIDForward.setSetpoint(target);
        } else if (orientation.toLowerCase().equals("sideways")) {
            PIDSideways.setSetpoint(target);
        } else if (orientation.toLowerCase().equals("rotation")) {
            PIDRotation.setSetpoint(target);
        } 
        // else if (orientation.toLowerCase().equals("distancetx")) {
        //     PIDDistanceTx.setSetpoint(target);
        // }
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
