// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/* 
 * Code does not work as intended
 */

package frc.robot.commands.LimeLight;

import java.util.Arrays;

import frc.robot.Robot;
import frc.robot.subsystems.LimeVision.LimeLightSub;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.LimeLight.FaceAprilTag;

// drivetrain
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import swervelib.SwerveController;

/** An example command that uses an example subsystem. */
public class DriveToTag extends Command {

  private final LimeLightSub limeLightSub;

  private final SwerveSubsystem  swerve;
  private final BooleanSupplier  driveMode;
  private final SwerveController controller;

  private double targetDistanceFromX;
  private double targetDistanceFromY;
  private double targetDistanceFromRZ;
  private int zero_ct;

  private static boolean isFinished;
  /**
   * Creates a new ExampleCommand.
   *
   * @param swerve The subsystem used by this command.
   */
 
  public DriveToTag(SwerveSubsystem swerve, LimeLightSub limeLightSub, BooleanSupplier driveMode) {
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
    targetDistanceFromX = limeLightSub.getPoseX() - 1;
    targetDistanceFromY = 0;
    zero_ct = 0;
    limeLightSub.setSetpointX(targetDistanceFromX);
    limeLightSub.setSetpointY(targetDistanceFromY);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("works");
    System.out.println(limeLightSub.hasTarget());

    // increment if zero for stopping condition
    if(limeLightSub.hasTarget() == false) {
      zero_ct ++;
      return;
    } else {
      zero_ct = 0;
    }

    
    // Drive using raw values.
    // double xpos_sum = 0;
    // double ypos_sum = 0;
    // int iters = 5;
    // for(int i = 0; i < iters-1; i++){
    //   xpos_sum += -limeLightSub.getErrorFromMegaTagX() * swerve.maximumSpeed;
    //   ypos_sum += -limeLightSub.getErrorFromMegaTagY() * swerve.maximumSpeed;
    // }
    // double xpos = xpos_sum/iters;
    // double ypos = ypos_sum/iters;

    // System.out.println(xpos);
    
    // double xpos = -limeLightSub.getErrorFromMegaTagX() * swerve.maximumSpeed;
    // double ypos = -limeLightSub.getErrorFromMegaTagY() * swerve.maximumSpeed;

    // System.out.println(xpos);
    // System.out.println(ypos);
    // System.out.println("---");
    
    swerve.drive(new Translation2d(0.25, 0),
                 0 * controller.config.maxAngularVelocity,
                 driveMode.getAsBoolean());
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // limeLightSub.hasTarget() == false
    if ((limeLightSub.getErrorFromMegaTagX() < 0.5 && limeLightSub.getErrorFromMegaTagX() > -0.5) || zero_ct > 5) {
      return true;
      // if (limeLightSub.getErrorFromMegaTagY() < 0.3 && limeLightSub.getErrorFromMegaTagY() > -0.3) {
      //   return true;
      // }
    }
    return false;
  }
}