// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.driveCommands;

import java.util.function.DoubleSupplier;

import frc.robot.subsystems.LimeVision.ApriltagController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;


public class driveToPose extends Command {
  /** Creates a new rotateinPlace. */
  private final SwerveSubsystem m_Subsystem;
  private final ApriltagController apriltagController;

  // making this public so can access in shuffleboard in drivecontainer (will this work?)
  public static double angleRate = Double.POSITIVE_INFINITY;

  public driveToPose(SwerveSubsystem subsystem, ApriltagController apriltagController) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_Subsystem = subsystem;
    this.apriltagController = apriltagController;

    addRequirements(m_Subsystem, apriltagController);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d position = apriltagController.getTargetPose();
    System.out.println(position.getX());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // CommandScheduler.getInstance().schedule(m_Subsystem.driveToPose(apriltagController.getTargetPose()));
    // CommandScheduler.getInstance().schedule(m_Subsystem.driveToPose(apriltagController.getTargetPose()));
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
