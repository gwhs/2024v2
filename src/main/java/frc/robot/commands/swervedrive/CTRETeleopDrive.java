// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Util.UtilMath;
import frc.robot.subsystems.swervedrive.CommandSwerveDrivetrain;
import frc.robot.subsystems.swervedrive.TunerConstants;

public class CTRETeleopDrive extends Command {
  /** Creates a new CTRETeleopDrive. */
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 2.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  private CommandSwerveDrivetrain drivetrain = CommandSwerveDrivetrain.getInstance();

  public boolean isFaceSpeaker = false;
  public boolean isBackSpeaker = false;
  public boolean faceAmp = false;
  public boolean isSlow = false;
  public boolean isHeadingLock = false;
  public boolean faceSpeaker = false;
  private PIDController PID;
  private Pose2d currPose;

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop

  private CommandXboxController driverController;

  public CTRETeleopDrive(CommandXboxController driver) {
    // Use addRequirements() here to declare subsystem dependencies.
    driverController = driver;

    this.PID = new PIDController(Constants.DriveConstants.kP, Constants.DriveConstants.kI, Constants.DriveConstants.kD);
    this.PID.setTolerance(Constants.FaceSpeakerConstants.THETA_TOLERANCE, Constants.FaceSpeakerConstants.STEADY_STATE_TOLERANCE);
    this.PID.enableContinuousInput(-180, 180);
  
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xVelocity = -driverController.getLeftY() * MaxSpeed;
    double yVelocity = -driverController.getLeftX() * MaxSpeed;
    //double angularVelocity = -driverController.getRightX() * MaxAngularRate;
    double angularVelocity = (MathUtil.applyDeadband(driverController.getLeftTriggerAxis(), OperatorConstants.ROTATION_DEADBAND) - MathUtil.applyDeadband(driverController.getRightTriggerAxis(), OperatorConstants.ROTATION_DEADBAND));

    if(DriverStation.getAlliance().isPresent() && 
       DriverStation.getAlliance().get() == DriverStation.Alliance.Red)
    {
      xVelocity *= -1;
      yVelocity *= -1;
    }
    
    currPose = drivetrain.getState().Pose;
    double currTheta = currPose.getRotation().getDegrees();
    SmartDashboard.putNumber("Robot Rotation", currTheta);

    if(isSlow)
    {
      double slowFactor = 0.25;
      xVelocity *= slowFactor;
      yVelocity *= slowFactor;
      angularVelocity *= slowFactor;
    }

    if(isFaceSpeaker)
    {
      double ang = UtilMath.FrontSpeakerTheta(currPose);
      PID.setSetpoint(ang);
      angularVelocity = PID.calculate(currTheta);
      if(angularVelocity > Constants.DriveConstants.MAX_RANGE) {
        angularVelocity = Constants.DriveConstants.MAX_RANGE;
      }
      else if(angularVelocity < -Constants.DriveConstants.MAX_RANGE) {
        angularVelocity = -Constants.DriveConstants.MAX_RANGE;
      }
      SmartDashboard.putNumber("isFaceSpeaker Goal", ang);
      SmartDashboard.putNumber("isFaceSpeaker Result", angularVelocity);
    }
    else if(isBackSpeaker)
    {
      double ang = UtilMath.BackSpeakerTheta(currPose);
      PID.setSetpoint(ang);
      angularVelocity = PID.calculate(currTheta);
      if(angularVelocity > Constants.DriveConstants.MAX_RANGE) {
        angularVelocity = Constants.DriveConstants.MAX_RANGE;
      }
      else if(angularVelocity < -Constants.DriveConstants.MAX_RANGE) {
        angularVelocity = -Constants.DriveConstants.MAX_RANGE;
      }
      SmartDashboard.putNumber("isBackSpeaker Goal", ang);
      SmartDashboard.putNumber("isBackSpeaker Result", angularVelocity);
    }

    if (faceAmp) {
      PID.setSetpoint(-90);
      angularVelocity = PID.calculate(currTheta);
      if(angularVelocity > Constants.DriveConstants.MAX_RANGE) {
        angularVelocity = Constants.DriveConstants.MAX_RANGE;
      }
      else if(angularVelocity < -Constants.DriveConstants.MAX_RANGE) {
        angularVelocity = -Constants.DriveConstants.MAX_RANGE;
      }
      SmartDashboard.putNumber("faceAmp Goal", -90);
      SmartDashboard.putNumber("faceAmp Result", angularVelocity);
    }

    if (faceSpeaker)
    {
      double result = 0;
      if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue){
        PID.setSetpoint(-180);
      }
      else{
        PID.setSetpoint(0);
      }
      result = PID.calculate(currTheta);
     
      if(result > Constants.DriveConstants.MAX_RANGE){
        result = Constants.DriveConstants.MAX_RANGE;
      }
      else if(result < -Constants.DriveConstants.MAX_RANGE){
        result = -Constants.DriveConstants.MAX_RANGE;
      }
      angularVelocity += result;
    }

    if(isHeadingLock)
    {
      double theta = UtilMath.SourceIntakeHeading(currPose);
      PID.setSetpoint(theta);

      double result =  PID.calculate(currTheta);
      if(result > Constants.DriveConstants.MAX_RANGE) {
        result = Constants.DriveConstants.MAX_RANGE;
      }
      else if(result < -Constants.DriveConstants.MAX_RANGE) {
        result = -Constants.DriveConstants.MAX_RANGE;
      }
      angularVelocity += result;
      SmartDashboard.putNumber("heading Lock Goal", theta);
      SmartDashboard.putNumber("heading Lock Result", result);
    }
    double tempXVelocity = xVelocity;
    double tempYVelocity = yVelocity;
    double tempAngularVelocity = angularVelocity  * MaxAngularRate;
    SmartDashboard.putNumber("xVelocity", tempXVelocity);
    SmartDashboard.putNumber("yVelocity", tempYVelocity);
    SmartDashboard.putNumber("angularVelocity", tempAngularVelocity);
    // drivetrain.applyRequest(() -> drive.withVelocityX(tempXVelocity) // Drive forward with negative Y (forward)
    //         .withVelocityY(tempYVelocity) // Drive left with negative X (left)
    //         .withRotationalRate(tempAngularVelocity) // Drive counterclockwise with negative X (left)
    //     );
    drivetrain.setControl(drive.withVelocityX(tempXVelocity) // Drive forward with negative Y (forward)
             .withVelocityY(tempYVelocity) // Drive left with negative X (left)
             .withRotationalRate(tempAngularVelocity)); // Drive counterclockwise with negative X (left)
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
