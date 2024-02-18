// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.driveCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Util.UtilMath;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class toTrap extends Command {
  /** Creates a new toTrap. */
  private final SwerveSubsystem m_Subsystem;
  private static int bestTrapID;
  private static double bestTrap_X;
  private static double bestTrap_Y;
  private static Pose2d targetPose;
  public toTrap(SwerveSubsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_Subsystem = subsystem;
    addRequirements(m_Subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    bestTrapID = UtilMath.bestTrap(m_Subsystem.getPose());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(bestTrapID == 16)
    {
      System.out.println("16 is best");
      bestTrap_X = Constants.FieldConstants.BLUE_TRAP_16_X;
      bestTrap_Y = Constants.FieldConstants.BLUE_TRAP_16_Y;
      
      Translation2d targetTranslation = new Translation2d(Constants.FieldConstants.BLUE_TRAP_16_X, Constants.FieldConstants.BLUE_TRAP_16_Y);
      targetPose = new Pose2d(targetTranslation, new Rotation2d());
      m_Subsystem.driveToPose(targetPose);
    }
    else if(bestTrapID == 15)
    {
       System.out.println("15 is best");
      bestTrap_X = Constants.FieldConstants.BLUE_TRAP_15_X;
      bestTrap_Y = Constants.FieldConstants.BLUE_TRAP_15_Y;

      Translation2d targetTranslation = new Translation2d(Constants.FieldConstants.BLUE_TRAP_15_X, Constants.FieldConstants.BLUE_TRAP_15_Y);
      targetPose = new Pose2d(targetTranslation, new Rotation2d());
      m_Subsystem.drive(targetTranslation, 0, true);
    }
    else
    {
       System.out.println("14 is best");
      bestTrap_X = Constants.FieldConstants.BLUE_TRAP_14_X;
      bestTrap_Y = Constants.FieldConstants.BLUE_TRAP_14_Y;

       Translation2d targetTranslation = new Translation2d(Constants.FieldConstants.BLUE_TRAP_14_X, Constants.FieldConstants.BLUE_TRAP_14_Y);
      targetPose = new Pose2d(targetTranslation, new Rotation2d());
      m_Subsystem.drive(targetTranslation, 0, true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if( (m_Subsystem.getPose().getY() <= bestTrap_Y + 5 && m_Subsystem.getPose().getY() >= bestTrap_Y - 5) 
    || (m_Subsystem.getPose().getX() <= bestTrap_Y + 5 && m_Subsystem.getPose().getX() >= bestTrap_X - 5) )
    {
      return true;
    }
    return false;
  }
}
