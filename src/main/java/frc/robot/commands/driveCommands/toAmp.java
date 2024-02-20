package frc.robot.commands.driveCommands;

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
    private static double amp_X;
    private static double amp_Y;
    private static double targetX;
    private static double targetY;

    public toAmp(SwerveSubsystem subsystem) {
      // Use addRequirements() here to declare subsystem dependencies.
      this.m_Subsystem = subsystem;
      addRequirements(m_Subsystem);
    }

  // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        ampID = UtilMath.whichAmp(m_Subsystem.getPose());
    }
    
  // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    if (ampID == 6){
        amp_X = Constants.FieldConstants.BLUE_AMP_6_X;
        amp_Y = Constants.FieldConstants.BLUE_AMP_6_Y;
        
        targetX = Constants.FieldConstants.BLUE_AMP_6_X;
        targetY = Constants.FieldConstants.BLUE_AMP_6_Y - 1;;
    }
    else if (ampID == 5){
        amp_X = Constants.FieldConstants.RED_AMP_5_X;
        amp_Y = Constants.FieldConstants.RED_AMP_5_Y;

        targetX = Constants.FieldConstants.RED_AMP_5_X;
        targetY = Constants.FieldConstants.RED_AMP_5_Y - 1;;
    }
    Translation2d targetTranslation = new Translation2d(targetX - m_Subsystem.getPose().getX(), targetY - m_Subsystem.getPose().getY());
    m_Subsystem.drive(targetTranslation, 0, true);
    }
  // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    
    }

  // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if(ampID == 5 && (amp_Y -1 >= m_Subsystem.getPose().getY()))
        {
          return true;
        }
        else if(ampID == 6 && (amp_Y - 0.9 >= m_Subsystem.getPose().getY()))
        {
          return true;
        }
          return false;
        }
}
