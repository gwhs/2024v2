// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Util.UtilMath;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import swervelib.SwerveController;

/**
 * An example command that uses an example subsystem.
 */
public class TeleopDrive extends Command
{

  private final SwerveSubsystem  swerve;
  private final DoubleSupplier   vX;
  private final DoubleSupplier   vY;
  private final DoubleSupplier   omega;
  public BooleanSupplier  driveMode;
  private final SwerveController controller;
  public boolean isFaceSpeaker = false;
  public boolean isBackSpeaker = false;
  public boolean isSlow;
  public boolean isHeadingLock;
  private final PIDController PID;
  private double currTheta;

  /**
   * Creates a new ExampleCommand.
   *
   * @param swerve The subsystem used by this command.
   */
  public TeleopDrive(SwerveSubsystem swerve, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier omega,
                     BooleanSupplier driveMode)
  {
    this.swerve = swerve;
    this.vX = vX;
    this.vY = vY;
    this.omega = omega;
    this.driveMode = driveMode;
    this.controller = swerve.getSwerveController();

    this.PID = new PIDController(Constants.DriveConstants.kP, Constants.DriveConstants.kI, Constants.DriveConstants.kD);
    this.PID.setTolerance(Constants.FaceSpeakerConstants.THETA_TOLERANCE, Constants.FaceSpeakerConstants.STEADY_STATE_TOLERANCE);
    this.PID.setPID(Constants.FaceSpeakerConstants.kP, Constants.FaceSpeakerConstants.kI, Constants.FaceSpeakerConstants.kD);
    this.PID.enableContinuousInput(-180, 180);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
      isFaceSpeaker = false;
      isBackSpeaker = false;
      isSlow = false;
      isHeadingLock = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
     double xVelocity   = Math.pow(vX.getAsDouble(), 3);
     double yVelocity   = Math.pow(vY.getAsDouble(), 3);
     double angVelocity = Math.pow(omega.getAsDouble(), 3);
    if(DriverStation.getAlliance().isPresent() &&
     DriverStation.getAlliance().get() == DriverStation.Alliance.Red)
    {
      xVelocity *= -1;
      yVelocity *= -1;
    }
    
   
    
    currTheta = swerve.getHeading().getDegrees();

    // Drive using raw values.
    if(isFaceSpeaker)
    {
      PID.setSetpoint(UtilMath.FrontSpeakerTheta(swerve.getPose()));
      angVelocity = PID.calculate(currTheta);
    }
    else if(isBackSpeaker)
    {
      PID.setSetpoint(UtilMath.BackSpeakerTheta(swerve.getPose()));
      angVelocity = PID.calculate(currTheta);
    }

    if(isSlow)
    {
      double slowFactor = 0.25;
      xVelocity *= slowFactor;
      yVelocity *= slowFactor;
      angVelocity *= slowFactor;
    }

    if(isHeadingLock)
    {
       PID.setSetpoint(UtilMath.SourceIntakeHeading(swerve.getPose()));
       double result =  PID.calculate(currTheta)/4;
       angVelocity += result;
       SmartDashboard.putNumber("heading Lock Result", result);
    }


    swerve.drive(new Translation2d(xVelocity * swerve.maximumSpeed, yVelocity * swerve.maximumSpeed),
                angVelocity * controller.config.maxAngularVelocity,
                driveMode.getAsBoolean());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
  }

  // Returns true when the command ould end.
  @Override
  public boolean isFinished()
  {
    return false;
  }
}
