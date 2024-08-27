// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.driveCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.swervedrive.drivebase.TeleopDrive;

public class ResetTeleopDrive extends Command {
  /** Creates a new ResetTeleopDrive. */
  private TeleopDrive teleop; 
  public ResetTeleopDrive(TeleopDrive t) {
    teleop = t;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    teleop.isFaceSpeaker = false;
    teleop.isBackSpeaker = false;
    teleop.isSlow = false;
    teleop.faceAmp = false;
    teleop.faceSpeaker = false;
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
