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
  private static double targetX;
  private static double targetY;
  public toTrap(SwerveSubsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_Subsystem = subsystem;
    addRequirements(m_Subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    bestTrapID = UtilMath.BlueBestTrap(m_Subsystem.getPose());
    if(bestTrapID == 16)
    {
      // System.out.println("16 is best");
      bestTrap_X = Constants.FieldConstants.BLUE_TRAP_16_X;
      bestTrap_Y = Constants.FieldConstants.BLUE_TRAP_16_Y;

      targetX = bestTrap_X - Math.sin(Math.PI/6)*0.7;
      targetY = bestTrap_Y - Math.cos(Math.PI/6)*0.7;
    }
    else if(bestTrapID == 15)
    {
      //  System.out.println("15 is best");
      bestTrap_X = Constants.FieldConstants.BLUE_TRAP_15_X;
      bestTrap_Y = Constants.FieldConstants.BLUE_TRAP_15_Y;

      targetX = bestTrap_X - Math.sin(Math.PI/6)*0.7;
      targetY = bestTrap_Y + Math.cos(Math.PI/6)*0.7;;

    }
    else if(bestTrapID == 14)
     {
    //    System.out.println("14 is best");
      bestTrap_X = Constants.FieldConstants.BLUE_TRAP_14_X;
      bestTrap_Y = Constants.FieldConstants.BLUE_TRAP_14_Y;

      targetX = bestTrap_X + 0.7;
      targetY = bestTrap_Y;

    }
    else if(bestTrapID == 11)
    {
      bestTrap_X = Constants.FieldConstants.RED_TRAP_11_X;
      bestTrap_Y = Constants.FieldConstants.RED_TRAP_11_Y;

      targetX = bestTrap_X + Math.sin(Math.PI/6)*0.7;
      targetY = bestTrapID + Math.cos(Math.PI/6)*0.7;
    }
    else if(bestTrapID == 12)
    {
      bestTrap_X = Constants.FieldConstants.RED_TRAP_12_X;
      bestTrap_Y = Constants.FieldConstants.RED_TRAP_12_Y;

      targetX = bestTrap_X + Math.sin(Math.PI/6)*0.7;
      targetY = bestTrap_Y - Math.cos(Math.PI/6)*0.7;
    }
    else if(bestTrapID == 13)
    {
      bestTrap_X = Constants.FieldConstants.RED_TRAP_13_X;
      bestTrap_Y = Constants.FieldConstants.RED_TRAP_13_Y;

      targetX = bestTrap_X - 0.7;
      targetY = bestTrap_Y;
    }
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     
        Translation2d targetTranslation = new Translation2d(targetX - m_Subsystem.getPose().getX(), targetY - m_Subsystem.getPose().getY());

       m_Subsystem.drive(targetTranslation, 0, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
     if(bestTrapID == 14 && (bestTrap_X -1 <= m_Subsystem.getPose().getX() && m_Subsystem.getPose().getX() <= bestTrap_X + 1))
    {
      return true;
    }
    else if(bestTrapID == 13 && (bestTrap_X - 0.9 <= m_Subsystem.getPose().getX() && m_Subsystem.getPose().getX() <= bestTrap_X + 0.9))
    {
      return true;
    }
    else if(bestTrapID != 13 && bestTrapID != 14 && bestTrap_Y - 1 <= m_Subsystem.getPose().getY() && m_Subsystem.getPose().getY() <= bestTrap_Y + 1)
    {
      return true;
    }
   
    return false;
  }
}
