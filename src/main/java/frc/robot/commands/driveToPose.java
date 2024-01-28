// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;


public class driveToPose extends Command {
  /** Creates a new driveToPose. */
  private final SwerveSubsystem m_Subsystem;
  private final Pose2d posObj;

  public driveToPose(double x, double y, Rotation2d rotation, SwerveSubsystem swerveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    double xPos = x;
    double yPos = y;
    Rotation2d rot = rotation;
    posObj = new Pose2d(xPos, yPos, rot);
    m_Subsystem = swerveSubsystem;
    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    CommandScheduler.getInstance().schedule(m_Subsystem.driveToPose(posObj));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   

  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return true;
  }
}
