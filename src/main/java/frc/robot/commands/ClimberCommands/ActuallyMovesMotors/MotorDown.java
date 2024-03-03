// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimberCommands.ActuallyMovesMotors;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.Climbsubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class MotorDown extends Command {
  /** Creates a new MotorDown. */

  private Climbsubsystem climbersubsystem;
  private SwerveSubsystem swerve;

  // private final double CLIMBER_PID_KP = 1.9;
  // private final double CLIMBER_PID_KI = 0;
  // private final double CLIMBER_PID_KD = 0;
  // private Constraints constraints = new Constraints(180.0, 300.0);

  // private ProfiledPIDController leftPIDcontroller = new ProfiledPIDController(CLIMBER_PID_KP, CLIMBER_PID_KI, CLIMBER_PID_KD, constraints); 
  // private ProfiledPIDController rightPIDcontroller = new ProfiledPIDController(CLIMBER_PID_KP, CLIMBER_PID_KI, CLIMBER_PID_KD, constraints); 


  //constructor that takes in a Climbsubsystem object and a SwerveSubsystem obj
  public MotorDown(Climbsubsystem c, SwerveSubsystem s) {

    climbersubsystem = c;
    swerve = s;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.climbersubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // leftPIDcontroller.setGoal(-0.3);
    // rightPIDcontroller.setGoal(0.3);
    climbersubsystem.downMotor();
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
    
    double leftDelta = -climbersubsystem.getPositionLeft() - 0.25;
    double rightDelta  = climbersubsystem.getPositionRight() - 0.25;
    return (climbersubsystem.getBotLeftLimit() && climbersubsystem.getBotRightLimit())
            || (Math.abs(leftDelta) < 0.75 && Math.abs(rightDelta) < 0.75); 
  }
}
