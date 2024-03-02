// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.driveCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.swervedrive.drivebase.TeleopDrive;

public class BackSpeaker extends Command {
  private TeleopDrive drivebase;
  /** Creates a new BackSpeaker. */
  public BackSpeaker(TeleopDrive drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    drivebase = drive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivebase.isBackSpeaker = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivebase.isBackSpeaker = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
