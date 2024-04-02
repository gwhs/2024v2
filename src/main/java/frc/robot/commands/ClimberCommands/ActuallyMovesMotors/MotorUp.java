// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimberCommands.ActuallyMovesMotors;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Climbsubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class MotorUp extends Command {
  /** Creates a new MotorUp. */

  private final Climbsubsystem climbersubsystem;
  private final ArmSubsystem armsubsystem;

  //constructor that takes in a Climbsubsystem object and a SwerveSubsystem obj
  public MotorUp(Climbsubsystem c, ArmSubsystem a) {

    climbersubsystem = c;
    armsubsystem = a;

    addRequirements(this.climbersubsystem);
  }

  @Override
  public void initialize() {
    System.out.println("Initialize motor up");
    climbersubsystem.upMotor();
  }

  @Override
  public void execute() {
    climbersubsystem.armAngleCheck = armsubsystem.checkEncoderAngleForClimb(); 
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("finish motor up");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //stops when reaches desired height
    //TO-DO: Use Constants
    double leftDelta = -climbersubsystem.getPositionLeft() - ClimbConstants.CLIMB_DISTANCE;
    double rightDelta  = climbersubsystem.getPositionRight() - ClimbConstants.CLIMB_DISTANCE;
    return (climbersubsystem.getTopLeftLimit() && climbersubsystem.getTopRightLimit())                                         
            || (Math.abs(leftDelta) < 20 && Math.abs(rightDelta) < 20); 
  }
}