// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimberCommands.ClimbParts;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climbsubsystem;

public class StopClimb extends Command {
  /** Creates a new StopClimb. */
  private Climbsubsystem climbSubsystem;
  public StopClimb(Climbsubsystem c) {
    climbSubsystem = c;
    // Use addRequirements() here to declare subsystem dependencies.
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climbSubsystem.stopClimbMotorInCaseOfEmergencySoThisWillStopTheClimbNoMatterIfItIsGoingUpOrDown();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
