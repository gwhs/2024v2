package frc.robot.subsystems.LimeVision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;


public class ApriltagController extends SubsystemBase {

    private final SwerveSubsystem swerve;
    private final LimeLightSub limeLightSub;

        // PID constants
    private final double kPX = 0.3;
    private final double kDX = 0.1;
    private final double kIX = 0;

    //P - 0.6 sweet spot
    private final double kPY = 1;
    private final double kDY = 0;
    private final double kIY = 0;

    /* PID constants for faceAprilTag
    * P = 0.04, D = 0, I = 0
    */
    private final double kPTheta = 0.4;
    private final double kDTheta = 0;
    private final double kITheta = 0;
    
    private PIDController PIDVisionX = new PIDController(kPX, kIX, kDX);
    private PIDController PIDVisionY = new PIDController(kPY, kIY, kDY);
    private PIDController PIDVisionTheta = new PIDController(kPTheta, kITheta, kDTheta);

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

    public ApriltagController(SwerveSubsystem swerve, LimeLightSub limeLightSub) {
        this.swerve = swerve;
        this.limeLightSub = limeLightSub;
    }

    public double getDistanceX() {
        return apriltag[limeLightSub.getID()][0] - swerve.getPose().getX();
    }

    public double getDistanceY() {
        return apriltag[limeLightSub.getID()][1] - swerve.getPose().getY();
    }
    

    public double getErrorX() {  // front and back
        return PIDVisionX.calculate(getDistanceX());
    }
      
    public double getErrorY() {  // left and right
        return PIDVisionY.calculate(getDistanceY());
    }

    public double getThetaError() {
        return PIDVisionTheta.calculate(limeLightSub.getTx());
    }

    public double getSmoothThetaError() {
        return PIDVisionTheta.calculate(limeLightSub.getSmoothTheta());
    }



  // setsPoint PID
    public void setPoint(double target, String orientation) {
        if (orientation.toLowerCase().equals("x")) {
            PIDVisionX.setSetpoint(target);
        } else if (orientation.toLowerCase().equals("y")) {
            PIDVisionY.setSetpoint(target);
        } else if (orientation.toLowerCase().equals("theta")) {
            PIDVisionTheta.setSetpoint(target);
        }
    }
      
}
