// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimberCommands.ActuallyMovesMotors;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Climbsubsystem;

public class MotorDown extends Command {
  /** Creates a new MotorDown. */

  private Climbsubsystem climbersubsystem;
  private ArmSubsystem armsubsystem;

  //constructor that takes in a Climbsubsystem object and a SwerveSubsystem obj
  public MotorDown(Climbsubsystem c, ArmSubsystem a) {
   
    climbersubsystem = c;
    armsubsystem = a;

    addRequirements(this.climbersubsystem);
  }

  @Override
  public void initialize() {
    climbersubsystem.downMotor();
  }

  @Override
  public void execute() {   
    climbersubsystem.armAngleCheck = armsubsystem.checkEncoderAngleForClimb();            
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    //TO-DO: instead of 0.25, use constants.
    double leftDelta = -climbersubsystem.getPositionLeft() - 0.25;
    double rightDelta  = climbersubsystem.getPositionRight() - 0.25;
    return (climbersubsystem.getBotLeftLimit() && climbersubsystem.getBotRightLimit())
            || (Math.abs(leftDelta) < 0.75 && Math.abs(rightDelta) < 0.75); 
  }
}
