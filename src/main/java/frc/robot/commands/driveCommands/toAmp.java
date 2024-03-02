package frc.robot.commands.driveCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Util.UtilMath;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class toAmp extends Command {
    private final SwerveSubsystem m_Subsystem;
    private static int ampID;
    private static double targetX;
    private static double targetY;
    private static PIDController PIDx;
    private static PIDController PIDy;
    private static Translation2d pose;

    public toAmp(SwerveSubsystem subsystem) {
      // Use addRequirements() here to declare subsystem dependencies.
      this.m_Subsystem = subsystem;
      PIDx = new PIDController(Constants.toAmpPID.kPx, Constants.toAmpPID.kIx, Constants.toAmpPID.kDx);
      PIDy = new PIDController(Constants.toAmpPID.kPy, Constants.toAmpPID.kIy, Constants.toAmpPID.kDy);
      addRequirements(m_Subsystem);
    }

   //public toAmp(Object subsystem) {
        //TODO Auto-generated constructor stub
   //}

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        ampID = UtilMath.whichAmp(m_Subsystem.getPose());
        if (ampID == 6){
        targetX = Constants.FieldConstants.BLUE_AMP_6_X;
        targetY = Constants.FieldConstants.BLUE_AMP_6_Y - UtilMath.inchesToMeters(0.5);
    }
    else if (ampID == 5){
        targetX = Constants.FieldConstants.RED_AMP_5_X;
        targetY = Constants.FieldConstants.RED_AMP_5_Y - UtilMath.inchesToMeters(0.5);
    }
       pose = new Translation2d(targetX, targetY);
    }
    
  // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    m_Subsystem.drive(pose, 0, true);
    }
  // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

  // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if(ampID == 5 && (targetY -1 <= m_Subsystem.getPose().getY() && m_Subsystem.getPose().getY() <= targetY + 1))
        {
          return true;
        }
        else if(ampID == 6 && (targetY -1 <= m_Subsystem.getPose().getY() && m_Subsystem.getPose().getY() <= targetY + 1))
        {
          return true;
        }
          return false;
        }
      }
