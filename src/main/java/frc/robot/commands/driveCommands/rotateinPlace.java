// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.driveCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Util.UtilMath;
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
  private double marginOfError = 1;
  private double exceptionTarget = 180;
  private double exceptionTMarginOfError = 175;

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
    currTheta = m_Subsystem.getHeading().getDegrees();
    double dif = targetTheta - currTheta;
      if( !(dif < 0 ^ Math.abs(dif) < 180) ){
        spinRate *= -1;
      }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currTheta = m_Subsystem.getHeading().getDegrees();
     m_Subsystem.drive(pose, spinRate, false);
    diff =Math.abs(currTheta - targetTheta);
    }
    
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double currTheta = m_Subsystem.getHeading().getDegrees();
    if(targetTheta == exceptionTarget && (currTheta >= exceptionTMarginOfError || currTheta <= -1*(exceptionTMarginOfError)))
    {
      return true;
    }
    else
    {
      if((diff <= marginOfError))
      {
        return true;
      }

    }
    return false;

  }
}
