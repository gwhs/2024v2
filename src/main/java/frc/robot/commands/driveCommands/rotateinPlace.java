// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.driveCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class rotateinPlace extends Command {
  /** Creates a new rotateinPlace. */
  private final SwerveSubsystem m_Subsystem;
  private final Translation2d pose;
  private double spinRate = Math.PI/3;
  private final DoubleSupplier targetTDoubleSupplier;
  private double targetTheta;
  private double currTheta;
  private double diff;

  public rotateinPlace(DoubleSupplier rotation, SwerveSubsystem subsystem ) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_Subsystem = subsystem;
    this.targetTDoubleSupplier = rotation;

    
    pose = new Translation2d();

    addRequirements(m_Subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currTheta = m_Subsystem.getHeading().getDegrees();
    double rotation = targetTDoubleSupplier.getAsDouble();
    if (rotation > 180)
    { double difference = rotation - 180;
      rotation = -180 + difference;
    }
    this.targetTheta = rotation;
      if(targetTheta < 0 ){
        spinRate *= -1;
      }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currTheta = m_Subsystem.getHeading().getDegrees();
     m_Subsystem.drive(pose, spinRate, false);
    diff = Math.abs(currTheta - targetTheta);
    }
    
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double currTheta = m_Subsystem.getHeading().getDegrees();
    if(targetTheta == 180 && (currTheta >= 175 || currTheta <= -175))
    {
      return true;
    }
    else
    {
      
      System.out.println(diff);
      if((diff <= 5))
      {
        return true;
      }

    }
    return false;

  }
}
