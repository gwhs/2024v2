// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimberCommands.ActuallyMovesMotors;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.Climbsubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class MotorUp extends Command {
  /** Creates a new MotorUp. */

  private final Climbsubsystem climbersubsystem;
  private final SwerveSubsystem swerve;

  // private final double CLIMBER_PID_KP = 1.9;
  // private final double CLIMBER_PID_KI = 0;
  // private final double CLIMBER_PID_KD = 0;
  // private Constraints constraints = new Constraints(180.0, 300.0);

  // private ProfiledPIDController leftPIDcontroller = new ProfiledPIDController(CLIMBER_PID_KP, CLIMBER_PID_KI, CLIMBER_PID_KD, constraints); 
  // private ProfiledPIDController rightPIDcontroller = new ProfiledPIDController(CLIMBER_PID_KP, CLIMBER_PID_KI, CLIMBER_PID_KD, constraints); 

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
    //climbersubsystem.upMotor();
    // leftPIDcontroller.setGoal(-198.94);
    // rightPIDcontroller.setGoal(198.4);
    climbersubsystem.upMotor();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // double leftPIDvalue = leftPIDcontroller.calculate(climbersubsystem.getPositionLeft());
    // double rightPIDvalue = rightPIDcontroller.calculate(climbersubsystem.getPositionRight());

    // climbersubsystem.setSpeed(-leftPIDvalue, rightPIDvalue);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //stops when reaches desired height
    double leftDelta = -climbersubsystem.getPositionLeft() - ClimbConstants.CLIMB_DISTANCE;
    double rightDelta  = climbersubsystem.getPositionRight() - ClimbConstants.CLIMB_DISTANCE;
    return (climbersubsystem.getTopLeftLimit() && climbersubsystem.getTopRightLimit())                                         
            || (Math.abs(leftDelta) < 20 && Math.abs(rightDelta) < 20); 
  }
}