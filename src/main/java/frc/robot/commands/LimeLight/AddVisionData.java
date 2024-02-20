package frc.robot.commands.LimeLight;

import frc.robot.subsystems.LimeVision.LimeLightSub;
import frc.robot.subsystems.LimelightHelpers.LimelightHelpers;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

import java.util.function.BooleanSupplier;


import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command; 

public class AddVisionData extends Command{

private final SwerveSubsystem swerve;
private final LimeLightSub limeLightSub;
private final int meterToStop = 10;



    
    public AddVisionData(SwerveSubsystem swerve, LimeLightSub limeLightSub ) {
        this.swerve = swerve;
        this.limeLightSub = limeLightSub;
        
        
    

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(swerve, limeLightSub);

  }

    // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double[] temp = limeLightSub.getBlueBotPose();
    Pose2d currentPose = swerve.getPose();
    double distance = Math.hypot( temp[0]- currentPose.getX(), temp[1]- currentPose.getY());

    
    System.out.println("working");

    if(limeLightSub.hasTarget()){
       
        double xyStds= 0;
        double degStds = 0;
        Matrix<N3, N1> stds = new Matrix<N3, N1>(Nat.N3(), Nat.N1());
        int tagsVisable = LimelightHelpers.getLatestResults("limelight").targetingResults.targets_Fiducials.length;
        if(tagsVisable >= 2 ){
            xyStds = 0.5;
            degStds = 6;
            // System.out.println(xyStds);
            // System.out.println(degStds);
            SmartDashboard.putNumber("xyStds", xyStds);
            SmartDashboard.putNumber("degStds", degStds);
        }
        else if ((temp[8] < meterToStop) && (distance < meterToStop)) {
            xyStds = 1.0;
            degStds = 12;
            // System.out.println(xyStds);
            // System.out.println(degStds);
            SmartDashboard.putNumber("xyStds", xyStds);
            SmartDashboard.putNumber("degStds", degStds);
            
        }
          stds.set(0,0,xyStds);
          stds.set(1,0,xyStds);
          stds.set(2,0, degStds);
          Rotation2d degree = new Rotation2d(temp[5]* Math.PI / 180);
          Pose2d newPose = new Pose2d(temp[0],temp[1],degree);
          
          swerve.addActualVisionReading(newPose ,Timer.getFPGATimestamp() - (temp[6]/1000.0),stds);
          
    }
}
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   return false;
  }
}
