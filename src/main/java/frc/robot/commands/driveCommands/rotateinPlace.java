// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.driveCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;


public class rotateinPlace extends Command {
  /** Creates a new rotateinPlace. */
  private final SwerveSubsystem m_Subsystem;
  private final Translation2d pose;
  private final DoubleSupplier targetTDoubleSupplier;
  private double targetTheta;
  private double currTheta;
  private PIDController PID;

  // making this public so can access in shuffleboard in drivecontainer (will this work?)
  public static double angleRate;

  public rotateinPlace(DoubleSupplier rotation, SwerveSubsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_Subsystem = subsystem;
    this.targetTDoubleSupplier = rotation;
     PID = new PIDController(Constants.DriveConstants.kP, Constants.DriveConstants.kI, Constants.DriveConstants.kD);

    
    pose = new Translation2d();

    addRequirements(m_Subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targetTheta = targetTDoubleSupplier.getAsDouble();
    currTheta = m_Subsystem.getHeading().getDegrees();
    PID.setSetpoint(targetTheta);
    PID.setTolerance(Constants.DriveConstants.THETA_TOLERANCE, Constants.DriveConstants.STEADY_STATE_TOLERANCE);
    PID.setPID(Constants.DriveConstants.kP, Constants.DriveConstants.kI, Constants.DriveConstants.kD);
    PID.enableContinuousInput(-180, 180);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currTheta = m_Subsystem.getHeading().getDegrees();

    angleRate = PID.calculate(currTheta);
    m_Subsystem.drive(pose, angleRate, true);
    SmartDashboard.putNumber("Error Rate", angleRate);
    }
    
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return PID.atSetpoint();
}

}
