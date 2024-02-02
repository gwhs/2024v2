// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.driveCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class RotateInPlace extends Command {
  /** Creates a new rotateinPlace. */
  private final SwerveSubsystem m_Subsystem;
  private final Translation2d pose;
  private final double spinRate = Math.PI/3;
  private final double targetTheta;
  private double currTheta;
  public RotateInPlace(double rotation, SwerveSubsystem subsystem ) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_Subsystem = subsystem;
    this.targetTheta = rotation;
    pose = new Translation2d();

    addRequirements(m_Subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     currTheta = m_Subsystem.getHeading().getDegrees();
    if(targetTheta == 180 && currTheta < 0)
    {
       m_Subsystem.drive(pose, spinRate, false);
    }
    else if(targetTheta == 180 && currTheta > 0)
    {
      m_Subsystem.drive(pose, -spinRate, false);
    }
    else
    {
      if(currTheta < 0 && 360 + currTheta < 180) 
      {
        m_Subsystem.drive(pose, spinRate, false);
      }
      else if(currTheta < 0 && 360 + currTheta > 180)
      {
        m_Subsystem.drive(pose, -spinRate, false);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(targetTheta == 180 && m_Subsystem.getHeading().getDegrees() >= 175 || m_Subsystem.getHeading().getDegrees() >= -175)
    {
      return true;
    }
    else
    {
      if(currTheta >= targetTheta - 5 || currTheta <= targetTheta + 5)
      {
        return true;
      }
    }
    return false;
  }
}
