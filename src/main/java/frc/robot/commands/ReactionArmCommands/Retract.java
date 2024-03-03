// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ReactionArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ReactionSubsystem;
import frc.robot.Constants;

public class Retract extends Command {
  /** Creates a new Retract. */
private final ReactionSubsystem m_Subsystem;
private double currPos; //position in rotation
private double endPos;
  public Retract(ReactionSubsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_Subsystem = subsystem;
    addRequirements(m_Subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currPos = m_Subsystem.getPos();
    endPos = Constants.ReactionConstants.retractedPosition; //adds the current position to the final which is the end pos.
    m_Subsystem.spinBackward();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currPos = m_Subsystem.getPos();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Subsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(currPos - endPos) < Constants.ReactionConstants.tolerance || currPos >0)
    {
      return true;
    }
    return false;

  }

}
