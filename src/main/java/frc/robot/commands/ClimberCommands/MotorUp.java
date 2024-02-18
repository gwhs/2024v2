// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.Climbsubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class MotorUp extends Command {
  /** Creates a new MotorUp. */

  private final Climbsubsystem climbersubsystem;
  private final SwerveSubsystem swerve;

  //constructor that takes in a Climbsubsystem object and a SwerveSubsystem obj
  public MotorUp(Climbsubsystem c, SwerveSubsystem s) {

    climbersubsystem = c;
    swerve = s;


    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.climbersubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftSpeed = ClimbConstants.CLIMB_MOTOR_SPEED;
    double rightSpeed = ClimbConstants.CLIMB_MOTOR_SPEED;

    //TEST THIS LATER
    if (swerve.getRoll().getDegrees() < -0.5) {
      leftSpeed += 2;
    } else if (swerve.getRoll().getDegrees() > 0.5) {
      rightSpeed += 2;
    }

    climbersubsystem.setSpeed(leftSpeed, rightSpeed); //sets the speed (in rotations/sec) to the value set in Constants file 
    //robot climbs down but motors go up so positive velocity
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // while (climbersubsystem.getTopLimit()) {
    //   climbersubsystem.setSpeed(ClimbConstants.CLIMB_MOTOR_SPEED/4, ClimbConstants.CLIMB_MOTOR_SPEED/4);
    // }
    climbersubsystem.stopClimb(); 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //stops when reaches desired height
    return climbersubsystem.getTopLimit();                                         
            //|| (climbersubsystem.getPositionLeft() >= ClimbConstants.CLIMB_DISTANCE || climbersubsystem.getPositionLeft() >= ClimbConstants.CLIMB_DISTANCE); 
  }
}
