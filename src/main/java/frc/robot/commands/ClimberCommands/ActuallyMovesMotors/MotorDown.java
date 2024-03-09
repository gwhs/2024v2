// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimberCommands.ActuallyMovesMotors;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climbsubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class MotorDown extends Command {
  /** Creates a new MotorDown. */

  private Climbsubsystem climbersubsystem;
  private SwerveSubsystem swerve;

  //constructor that takes in a Climbsubsystem object and a SwerveSubsystem obj
  public MotorDown(Climbsubsystem c, SwerveSubsystem s) {
    climbersubsystem = c;
    swerve = s;

    addRequirements(this.climbersubsystem);
  }

  @Override
  public void initialize() {
    System.out.println("Initialize motor down");
    climbersubsystem.downMotor();
  }

  @Override
  public void execute() {               
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("finish motor down");
  }

  @Override
  public boolean isFinished() {
    
    double leftDelta = -climbersubsystem.getPositionLeft() - 0.25;
    double rightDelta  = climbersubsystem.getPositionRight() - 0.25;
    return (climbersubsystem.getBotLeftLimit() && climbersubsystem.getBotRightLimit())
            || (Math.abs(leftDelta) < 0.75 && Math.abs(rightDelta) < 0.75); 
  }
}
