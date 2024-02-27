// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/* 
 * Code does not work as intended
 */

package frc.robot.commands.LimeLight;

import frc.robot.Robot;
import frc.robot.subsystems.LimeVision.LimeLightSub;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

// drivetrain
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import swervelib.SwerveController;

/** An example command that uses an example subsystem. */
public class Sideways extends Command {

  private final LimeLightSub limeLightSub;

  private final SwerveSubsystem  swerve;
  private final BooleanSupplier  driveMode;
  private final SwerveController controller;

  private boolean isFinished = false;


  /**
   * Creates a new ExampleCommand.
   *
   * @param swerve The subsystem used by this command.
   */
 
  public Sideways(SwerveSubsystem swerve, LimeLightSub limeLightSub, BooleanSupplier driveMode) {
    this.swerve = swerve;
    this.limeLightSub = limeLightSub;
    this.driveMode = driveMode;

    this.controller = swerve.getSwerveController();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve, limeLightSub);
  }  
 
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("works");
    double speedX = 0;
    double speedY = 0;
    double error = limeLightSub.getError();

    if (-limeLightSub.getBotPose()[0] < 5) {
      speedX = 2;
    } else {
      speedX = 0;
    }

    if (limeLightSub.getBotPose()[1] > 0.5) {
      speedY = 2;
    } else {
      speedY = 0;
    }

    if (!(-limeLightSub.getBotPose()[0] < 5) || !(limeLightSub.getBotPose()[1] > 0.5)) {
      isFinished = true;
    } else {
      isFinished = false;
    }

    // Drive using raw values.
    swerve.drive(new Translation2d(speedX * swerve.maximumSpeed, speedY * swerve.maximumSpeed),
                 error * controller.config.maxAngularVelocity,
                 driveMode.getAsBoolean());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}