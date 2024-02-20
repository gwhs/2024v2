// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.driveCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Util.UtilMath;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import edu.wpi.first.math.controller.PIDController;
import swervelib.parser.PIDFConfig;

public class rotateinPlace extends Command {
  /** Creates a new rotateinPlace. */
  private final SwerveSubsystem m_Subsystem;
  private final Translation2d pose;
  private final DoubleSupplier targetTDoubleSupplier;
  private double targetTheta;
  private double currTheta;
  private double kP = 1;
  private double kI = 0;
  private double kD = 0;
  private PIDController PID;
  private double angleRate;
  private double tolerance = 0.1; //in degrees

  public rotateinPlace(DoubleSupplier rotation, SwerveSubsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_Subsystem = subsystem;
    this.targetTDoubleSupplier = rotation;
     PID = new PIDController(kP, kI, kD);

    
    pose = new Translation2d();

    addRequirements(m_Subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targetTheta = targetTDoubleSupplier.getAsDouble();
    currTheta = m_Subsystem.getHeading().getDegrees();
    PID.setSetpoint(targetTheta);
    PID.setPID(kP, kI, kD);
    PID.enableContinuousInput(-180, 180);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currTheta = m_Subsystem.getHeading().getDegrees();

    angleRate = PID.calculate(currTheta);
    m_Subsystem.drive(pose, angleRate, true);
    }
    
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double currTheta = m_Subsystem.getHeading().getDegrees();

    boolean isCloseToTarget = Math.abs(PID.getPositionError()) >= tolerance;
    boolean isSteadyState = Math.abs(PID.getVelocityError()) >= tolerance;
    return isCloseToTarget && isSteadyState;
}

}
